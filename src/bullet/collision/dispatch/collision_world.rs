use glam::Vec3A;

use super::{
    collision_dispatcher::CollisionDispatcher,
    collision_object::{CollisionObject, CollisionObjectTypes},
};
use crate::bullet::{
    collision::{
        broadphase::{
            broadphase_proxy::{BroadphaseAabbCallback, BroadphaseProxy, CollisionFilterGroups},
            rs_broadphase::RsBroadphase,
        },
        narrowphase::persistent_manifold::{CONTACT_BREAKING_THRESHOLD, ContactAddedCallback},
        shapes::{triangle_callback::TriangleCallback, triangle_shape::TriangleShape},
    },
    dynamics::rigid_body::RigidBody,
    linear_math::{AffineExt, aabb_util_2::Aabb, interpolate_3},
};

// struct LocalShapeInfo {
//     shape_part: usize,
//     triangle_index: usize,
// }

pub struct LocalRayResult {
    collision_object_index: usize,
    // local_shape_info: LocalShapeInfo,
    hit_normal_world: Vec3A,
    hit_fraction: f32,
}

pub struct RayResultCallbackBase {
    pub closest_hit_fraction: f32,
    pub collision_object_index: Option<usize>,
    pub ignore_object_world_index: Option<usize>,
    pub collision_filter_group: i32,
    pub collision_filter_mask: i32,
    // pub flags: u32,
}

impl Default for RayResultCallbackBase {
    fn default() -> Self {
        Self {
            closest_hit_fraction: 1.0,
            collision_object_index: None,
            collision_filter_group: CollisionFilterGroups::DefaultFilter as i32,
            collision_filter_mask: CollisionFilterGroups::AllFilter as i32,
            ignore_object_world_index: None,
            // flags: 0,
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

struct SingleRayCallback<'a, T: RayResultCallback> {
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
    from: Vec3A,
    dir: Vec3A,
    dist: f32,
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
        let delta = from - to;
        let dist = delta.length();
        let hit_fraction = result_callback.get_base().closest_hit_fraction;

        Self {
            from,
            dist,
            hit_fraction,
            result_callback,
            collision_object,
            collision_object_index,
            dir: delta / dist,
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

impl<T: RayResultCallback> TriangleCallback for BridgeTriangleRaycastCallback<'_, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        _tri_aabb: &Aabb,
        _triangle_index: usize,
    ) -> bool {
        let dir_align = self.dir.dot(triangle.normal);
        if dir_align > -f32::EPSILON {
            return true;
        }

        let d = -triangle.normal.dot(triangle.points[0]);
        let normal_start = triangle.normal.dot(self.from);
        let t = -(normal_start + d) / dir_align;
        if t <= 0.0 {
            return true;
        }

        let p = self.from + self.dir * t;
        let inside_edges = triangle
            .normal
            .dot(triangle.edges[0].cross(p - triangle.points[0]))
            >= 0.0
            && triangle
                .normal
                .dot(triangle.edges[1].cross(p - triangle.points[1]))
                >= 0.0
            && triangle
                .normal
                .dot(triangle.edges[2].cross(p - triangle.points[2]))
                >= 0.0;
        if !inside_edges {
            return true;
        }

        let hit_fraction = t / self.dist;
        self.report_hit(triangle.normal, hit_fraction);

        true
    }
}

pub struct CollisionWorld {
    pub collision_objects: Vec<RigidBody>,
    pub dispatcher1: CollisionDispatcher,
    broadphase_pair_cache: RsBroadphase,
    num_skippable_statics: usize,
}

impl CollisionWorld {
    pub const fn new(dispatcher: CollisionDispatcher, pair_cache: RsBroadphase) -> Self {
        Self {
            collision_objects: Vec::new(),
            dispatcher1: dispatcher,
            broadphase_pair_cache: pair_cache,
            num_skippable_statics: 0,
        }
    }

    pub fn add_collision_object(
        &mut self,
        mut object: RigidBody,
        filter_group: i32,
        filter_mask: i32,
    ) -> usize {
        {
            let obj = &mut object.collision_object;
            obj.world_array_index = self.collision_objects.len();

            let trans = obj.get_world_transform();
            let aabb = obj.get_collision_shape().unwrap().get_aabb(trans);

            let proxy =
                self.broadphase_pair_cache
                    .create_proxy(aabb, obj, filter_group, filter_mask);

            obj.set_broadphase_handle(proxy);
        }

        let index = self.collision_objects.len();
        self.collision_objects.push(object);

        index
    }

    pub fn remove_collision_object(&mut self, world_index: usize) {
        self.collision_objects.remove(world_index);

        todo!("remove_collision_object not implemented");
    }

    fn update_aabbs(&mut self) {
        const CBT: Vec3A = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);

        let mut prev_is_static = true;
        for (i, rb) in self
            .collision_objects
            .iter()
            .enumerate()
            .skip(self.num_skippable_statics)
        {
            let col_obj = &rb.collision_object;
            debug_assert_eq!(col_obj.world_array_index, i);

            if prev_is_static && col_obj.is_static_object() {
                // static objects only need their aabbs set the first time
                self.num_skippable_statics += 1;
            } else {
                prev_is_static = false;
            }

            let mut aabb = col_obj
                .get_collision_shape()
                .as_ref()
                .unwrap()
                .get_aabb(col_obj.get_world_transform());

            aabb.min -= CBT;
            aabb.max += CBT;

            if col_obj.internal_type == CollisionObjectTypes::RigidBody as i32
                && !col_obj.is_static_or_kinematic_object()
            {
                let mut aabb2 = col_obj
                    .get_collision_shape()
                    .as_ref()
                    .unwrap()
                    .get_aabb(&col_obj.interpolation_world_transform);
                aabb2.min -= CBT;
                aabb2.max += CBT;
                aabb += aabb2;
            }

            debug_assert!(
                col_obj.is_static_object() || (aabb.max - aabb.min).length_squared() < 1e12
            );
            self.broadphase_pair_cache.set_aabb(
                col_obj,
                col_obj.get_broadphase_handle().unwrap(),
                aabb,
            );
        }
    }

    pub fn perform_discrete_collision_detection<T: ContactAddedCallback>(
        &mut self,
        contact_added_callback: &mut T,
    ) {
        self.update_aabbs();

        self.broadphase_pair_cache.calculate_overlapping_pairs();
        self.dispatcher1.dispatch_all_collision_pairs(
            &self.collision_objects,
            &mut self.broadphase_pair_cache,
            contact_added_callback,
        );
    }

    fn ray_test_single<T: RayResultCallback>(
        ray_from: Vec3A,
        ray_to: Vec3A,
        co: &CollisionObject,
        object_index: usize,
        result_callback: &mut T,
    ) {
        let world_to_co = co.get_world_transform().transpose();
        let ray_from_local = world_to_co.transform_point3a(ray_from);
        let ray_to_local = world_to_co.transform_point3a(ray_to);

        let mut rcb = BridgeTriangleRaycastCallback::new(
            ray_from_local,
            ray_to_local,
            result_callback,
            co,
            object_index,
        );
        co.get_collision_shape()
            .unwrap()
            .perform_raycast(&mut rcb, ray_from_local, ray_to_local);
    }

    pub fn ray_test<T: RayResultCallback>(
        &self,
        ray_from_world: Vec3A,
        ray_to_world: Vec3A,
        result_callback: &mut T,
    ) {
        let mut ray_cb =
            SingleRayCallback::new(ray_from_world, ray_to_world, self, result_callback);
        self.broadphase_pair_cache
            .ray_test(ray_from_world, ray_to_world, &mut ray_cb);
    }
}
