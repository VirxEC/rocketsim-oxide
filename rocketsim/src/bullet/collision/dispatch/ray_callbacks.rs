use glam::Vec3A;

use crate::bullet::{
    collision::{
        broadphase::{BroadphaseAabbCallback, BroadphaseProxy, CollisionFilterGroups},
        dispatch::{collision_object::CollisionObject, collision_world::CollisionWorld},
        shapes::{triangle_callback::TriangleRayCallback, triangle_shape::TriangleShape},
    },
    linear_math::interpolate_3,
};

pub struct LocalRayResult {
    collision_object_index: usize,
    hit_normal_world: Vec3A,
    hit_fraction: f32,
}

pub struct RayResultCallbackBase {
    pub closest_hit_fraction: f32,
    pub collision_object_index: Option<usize>,
    pub ignore_object_world_index: Option<usize>,
    pub collision_filter_group: u8,
    pub collision_filter_mask: u8,
}

impl Default for RayResultCallbackBase {
    fn default() -> Self {
        Self {
            closest_hit_fraction: 1.0,
            collision_object_index: None,
            collision_filter_group: CollisionFilterGroups::Default as u8,
            collision_filter_mask: CollisionFilterGroups::All as u8,
            ignore_object_world_index: None,
        }
    }
}

pub trait RayResultCallback {
    fn get_base(&self) -> &RayResultCallbackBase;
    fn has_hit(&self) -> bool {
        self.get_base().collision_object_index.is_some()
    }
    fn needs_collision(&self, proxy0: &BroadphaseProxy) -> bool {
        let base = self.get_base();
        if base.ignore_object_world_index == proxy0.client_object_idx {
            return false;
        }

        proxy0.collision_filter_group & base.collision_filter_mask != 0
            && base.collision_filter_group & proxy0.collision_filter_mask != 0
    }
    fn add_single_result(&mut self, ray_result: LocalRayResult) -> f32;
}

pub struct ClosestRayResultCallback {
    pub base: RayResultCallbackBase,
    ray_from_world: Vec3A,
    ray_to_world: Vec3A,
    pub hit_normal_world: Vec3A,
    pub hit_point_world: Vec3A,
}

impl ClosestRayResultCallback {
    pub fn new(
        ray_from_world: Vec3A,
        ray_to_world: Vec3A,
        ignore_object: &CollisionObject,
    ) -> Self {
        Self {
            base: RayResultCallbackBase {
                ignore_object_world_index: Some(ignore_object.world_array_index),
                ..Default::default()
            },
            ray_from_world,
            ray_to_world,
            hit_normal_world: Vec3A::ZERO,
            hit_point_world: Vec3A::ZERO,
        }
    }
}

impl RayResultCallback for ClosestRayResultCallback {
    #[inline]
    fn get_base(&self) -> &RayResultCallbackBase {
        &self.base
    }

    fn add_single_result(&mut self, ray_result: LocalRayResult) -> f32 {
        debug_assert!(ray_result.hit_fraction <= self.base.closest_hit_fraction);

        self.base.closest_hit_fraction = ray_result.hit_fraction;
        self.hit_normal_world = ray_result.hit_normal_world;

        self.base.collision_object_index = Some(ray_result.collision_object_index);
        self.hit_point_world = interpolate_3(
            self.ray_from_world,
            self.ray_to_world,
            ray_result.hit_fraction,
        );

        ray_result.hit_fraction
    }
}

pub struct SingleRayCallback<'a, T: RayResultCallback> {
    ray_from_world: Vec3A,
    ray_to_world: Vec3A,
    world: &'a CollisionWorld,
    result_callback: &'a mut T,
}

impl<'a, T: RayResultCallback> SingleRayCallback<'a, T> {
    pub const fn new(
        ray_from_world: Vec3A,
        ray_to_world: Vec3A,
        world: &'a CollisionWorld,
        result_callback: &'a mut T,
    ) -> Self {
        Self {
            ray_from_world,
            ray_to_world,
            world,
            result_callback,
        }
    }
}

impl<T: RayResultCallback> BroadphaseAabbCallback for SingleRayCallback<'_, T> {
    fn process(&mut self, proxy: &BroadphaseProxy) -> bool {
        if self.result_callback.get_base().closest_hit_fraction == 0.0 {
            return false;
        }

        let obj_index = proxy.client_object_idx.unwrap();
        let rb = &self.world.collision_objects[proxy.client_object_idx.unwrap()];
        let handle_idx = rb.collision_object.get_broadphase_handle().unwrap();
        let handle = &self.world.broadphase_pair_cache.handles[handle_idx].broadphase_proxy;

        if self.result_callback.needs_collision(handle) {
            CollisionWorld::ray_test_single(
                self.ray_from_world,
                self.ray_to_world,
                &rb.collision_object,
                obj_index,
                self.result_callback,
            );
        }

        true
    }
}

pub struct BridgeTriangleRaycastCallback<'a, T: RayResultCallback> {
    to: Vec3A,
    from: Vec3A,
    hit_fraction: f32,
    result_callback: &'a mut T,
    collision_object: &'a CollisionObject,
    collision_object_index: usize,
}

impl<'a, T: RayResultCallback> BridgeTriangleRaycastCallback<'a, T> {
    pub fn new(
        from: Vec3A,
        to: Vec3A,
        result_callback: &'a mut T,
        collision_object: &'a CollisionObject,
        collision_object_index: usize,
    ) -> Self {
        let hit_fraction = result_callback.get_base().closest_hit_fraction;

        Self {
            to,
            from,
            hit_fraction,
            result_callback,
            collision_object,
            collision_object_index,
        }
    }

    pub fn report_hit(&mut self, hit_normal_local: Vec3A, hit_fraction: f32) {
        if hit_fraction >= self.hit_fraction {
            return;
        }

        self.hit_fraction = hit_fraction;
        let hit_normal_world =
            self.collision_object.get_world_transform().matrix3 * hit_normal_local;

        let ray_result = LocalRayResult {
            hit_fraction,
            hit_normal_world,
            collision_object_index: self.collision_object_index,
        };
        self.result_callback.add_single_result(ray_result);
    }
}

impl<T: RayResultCallback> TriangleRayCallback for BridgeTriangleRaycastCallback<'_, T> {
    fn process_triangle(&mut self, triangle: &TriangleShape) -> f32 {
        const EDGE_TOLERANCE: f32 = -0.0001;

        let dist = triangle.points[0].dot(triangle.normal);
        let dist_a = triangle.normal.dot(self.from) - dist;
        let dist_b = triangle.normal.dot(self.to) - dist;
        if dist_a * dist_b >= 0.0 {
            return self.hit_fraction; // same sign
        }

        let proj_length = dist_a - dist_b;
        let distance = dist_a / proj_length;
        if distance >= self.hit_fraction {
            return self.hit_fraction;
        }

        let point = self.from.lerp(self.to, distance);
        let v0p = triangle.points[0] - point;
        let v1p = triangle.points[1] - point;
        let cp0 = v0p.cross(v1p);
        if cp0.dot(triangle.normal) < EDGE_TOLERANCE {
            return self.hit_fraction;
        }

        let v2p = triangle.points[2] - point;
        let cp1 = v1p.cross(v2p);
        if cp1.dot(triangle.normal) < EDGE_TOLERANCE {
            return self.hit_fraction;
        }

        let cp2 = v2p.cross(v0p);
        if cp2.dot(triangle.normal) < EDGE_TOLERANCE {
            return self.hit_fraction;
        }

        if dist_a <= 0.0 {
            self.report_hit(-triangle.normal, distance);
        } else {
            self.report_hit(triangle.normal, distance);
        }

        self.hit_fraction
    }
}
