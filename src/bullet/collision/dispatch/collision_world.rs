use super::{
    collision_dispatcher::CollisionDispatcher,
    collision_object::{CollisionObject, CollisionObjectTypes},
};
use crate::bullet::{
    collision::{
        broadphase::{
            broadphase_proxy::{BroadphaseAabbCallback, BroadphaseProxy, CollisionFilterGroups},
            dispatcher::DispatcherInfo,
            rs_broadphase::RsBroadphase,
        },
        narrowphase::persistent_manifold::{CONTACT_BREAKING_THRESHOLD, ContactAddedCallback},
        shapes::{
            collision_shape::CollisionShapes, triangle_callback::TriangleCallback,
            triangle_shape::TriangleShape,
        },
    },
    linear_math::interpolate_3,
};
use glam::{BVec3A, Vec3A};
use std::{cell::RefCell, rc::Rc};

// struct LocalShapeInfo {
//     shape_part: usize,
//     triangle_index: usize,
// }

pub struct LocalRayResult {
    collision_object: Rc<RefCell<CollisionObject>>,
    // local_shape_info: LocalShapeInfo,
    hit_normal_local: Vec3A,
    hit_fraction: f32,
}

pub struct RayResultCallbackBase<'a> {
    pub closest_hit_fraction: f32,
    pub collision_object: Option<Rc<RefCell<CollisionObject>>>,
    pub ignore_object: Option<&'a Rc<RefCell<CollisionObject>>>,
    pub collision_filter_group: i32,
    pub collision_filter_mask: i32,
    pub flags: u32,
}

impl Default for RayResultCallbackBase<'_> {
    fn default() -> Self {
        Self {
            closest_hit_fraction: 1.0,
            collision_object: None,
            collision_filter_group: CollisionFilterGroups::DefaultFilter as i32,
            collision_filter_mask: CollisionFilterGroups::AllFilter as i32,
            ignore_object: None,
            flags: 0,
        }
    }
}

pub trait RayResultCallback {
    fn get_base(&self) -> &RayResultCallbackBase<'_>;
    fn has_hit(&self) -> bool {
        self.get_base().collision_object.is_some()
    }
    fn needs_collision(&self, proxy0: &BroadphaseProxy) -> bool {
        let base = self.get_base();
        let collides = proxy0.collision_filter_group & base.collision_filter_mask != 0
            && base.collision_filter_group & proxy0.collision_filter_mask != 0;
        if let Some(ignore_obj) = base.ignore_object.as_ref()
            && let Some(client_obj) = proxy0.client_object.as_ref()
            && Rc::ptr_eq(client_obj, ignore_obj)
        {
            return false;
        }

        collides
    }
    fn add_single_result(&mut self, ray_result: LocalRayResult, normal_in_world_space: bool)
    -> f32;
}

pub struct ClosestRayResultCallback<'a> {
    pub base: RayResultCallbackBase<'a>,
    ray_from_world: Vec3A,
    ray_to_world: Vec3A,
    pub hit_normal_world: Vec3A,
    pub hit_point_world: Vec3A,
}

impl<'a> ClosestRayResultCallback<'a> {
    pub fn new(
        ray_from_world: Vec3A,
        ray_to_world: Vec3A,
        ignore_object: &'a Rc<RefCell<CollisionObject>>,
    ) -> Self {
        Self {
            base: RayResultCallbackBase {
                ignore_object: Some(ignore_object),
                ..Default::default()
            },
            ray_from_world,
            ray_to_world,
            hit_normal_world: Vec3A::ZERO,
            hit_point_world: Vec3A::ZERO,
        }
    }
}

impl RayResultCallback for ClosestRayResultCallback<'_> {
    fn get_base(&self) -> &RayResultCallbackBase<'_> {
        &self.base
    }

    fn add_single_result(
        &mut self,
        ray_result: LocalRayResult,
        normal_in_world_space: bool,
    ) -> f32 {
        debug_assert!(ray_result.hit_fraction <= self.base.closest_hit_fraction);

        self.base.closest_hit_fraction = ray_result.hit_fraction;
        self.hit_normal_world = if normal_in_world_space {
            ray_result.hit_normal_local
        } else {
            ray_result
                .collision_object
                .borrow()
                .get_world_transform()
                .matrix3
                * ray_result.hit_normal_local
        };

        self.base.collision_object = Some(ray_result.collision_object);
        self.hit_point_world = interpolate_3(
            self.ray_from_world,
            self.ray_to_world,
            ray_result.hit_fraction,
        );

        ray_result.hit_fraction
    }
}

struct SingleRayCallback<'a, T: RayResultCallback> {
    ray_direction_inverse: Vec3A,
    signs: BVec3A,
    lambda_max: f32,
    ray_from_world: Vec3A,
    ray_to_world: Vec3A,
    hit_normal: Vec3A,
    world: &'a CollisionWorld,
    result_callback: &'a mut T,
}

impl<'a, T: RayResultCallback> SingleRayCallback<'a, T> {
    pub fn new(
        ray_from_world: Vec3A,
        ray_to_world: Vec3A,
        world: &'a CollisionWorld,
        result_callback: &'a mut T,
    ) -> Self {
        let ray_dir = (ray_to_world - ray_from_world).normalize();
        let ray_direction_inverse = 1.0 / ray_dir;
        debug_assert!(!ray_direction_inverse.is_nan());

        Self {
            hit_normal: Vec3A::ZERO,
            signs: ray_direction_inverse.cmplt(Vec3A::ZERO),
            lambda_max: ray_dir.dot(ray_to_world - ray_from_world),
            ray_from_world,
            ray_to_world,
            world,
            result_callback,
            ray_direction_inverse,
        }
    }
}

impl<T: RayResultCallback> BroadphaseAabbCallback for SingleRayCallback<'_, T> {
    fn process(&mut self, proxy: &BroadphaseProxy) -> bool {
        if self.result_callback.get_base().closest_hit_fraction == 0.0 {
            return false;
        }

        let co = proxy.client_object.as_ref().unwrap();
        let handle_idx = co.borrow().get_broadphase_handle().unwrap();
        let handle = &self.world.broadphase_pair_cache.handles[handle_idx].broadphase_proxy;

        if self.result_callback.needs_collision(handle) {
            self.world.ray_test_single(
                self.ray_from_world,
                self.ray_to_world,
                co,
                self.result_callback,
            );
        }

        true
    }
}

pub struct BridgeTriangleRaycastCallback<'a, T: RayResultCallback> {
    from: Vec3A,
    to: Vec3A,
    dir: Vec3A,
    dist: f32,
    hit_fraction: f32,
    result_callback: &'a mut T,
    collision_object: &'a Rc<RefCell<CollisionObject>>,
}

impl<'a, T: RayResultCallback> BridgeTriangleRaycastCallback<'a, T> {
    pub fn new(
        from: Vec3A,
        to: Vec3A,
        result_callback: &'a mut T,
        collision_object: &'a Rc<RefCell<CollisionObject>>,
    ) -> Self {
        let delta = from - to;
        let dist = delta.length();

        Self {
            from,
            to,
            dist,
            result_callback,
            collision_object,
            dir: delta / dist,
            hit_fraction: 1.0,
        }
    }

    pub fn report_hit(&mut self, hit_normal_local: Vec3A, hit_fraction: f32) {
        let hit_normal_world =
            self.collision_object.borrow().get_world_transform().matrix3 * hit_normal_local;

        let ray_result = LocalRayResult {
            collision_object: self.collision_object.clone(),
            hit_fraction,
            hit_normal_local: hit_normal_world,
        };
        let normal_in_world_space = true;
        self.result_callback
            .add_single_result(ray_result, normal_in_world_space);
    }
}

impl<'a, T: RayResultCallback> TriangleCallback for BridgeTriangleRaycastCallback<'a, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        _tri_aabb_min: Vec3A,
        _tri_aabb_max: Vec3A,
        _part_id: usize,
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
        if hit_fraction < self.hit_fraction {
            self.hit_fraction = hit_fraction;
            self.report_hit(triangle.normal, hit_fraction);
        }

        true
    }
}

pub struct CollisionWorld {
    pub collision_objects: Vec<Rc<RefCell<CollisionObject>>>,
    pub dispatcher1: CollisionDispatcher,
    pub dispatcher_info: DispatcherInfo,
    broadphase_pair_cache: RsBroadphase,
    force_update_all_aabbs: bool,
}

impl CollisionWorld {
    pub fn new(dispatcher: CollisionDispatcher, pair_cache: RsBroadphase) -> Self {
        Self {
            collision_objects: Vec::new(),
            dispatcher1: dispatcher,
            dispatcher_info: DispatcherInfo::default(),
            broadphase_pair_cache: pair_cache,
            force_update_all_aabbs: true,
        }
    }

    pub fn add_collision_object(
        &mut self,
        object: Rc<RefCell<CollisionObject>>,
        filter_group: i32,
        filter_mask: i32,
    ) {
        object
            .borrow_mut()
            .set_world_array_index(self.collision_objects.len() as i32);

        let obj = object.borrow();
        let trans = obj.get_world_transform();

        let shape = obj.get_collision_shape().unwrap().borrow_mut();
        let (aabb_min, aabb_max) = shape.get_aabb(trans);

        drop(shape);
        drop(obj);

        let proxy = self.broadphase_pair_cache.create_proxy(
            aabb_min,
            aabb_max,
            object.clone(),
            filter_group,
            filter_mask,
        );

        object.borrow_mut().set_broadphase_handle(proxy);
        self.collision_objects.push(object);
    }

    fn update_single_aabb(&mut self, col_obj_idx: usize) {
        let col_obj = self.collision_objects[col_obj_idx].borrow();
        let (mut min_aabb, mut max_aabb) = col_obj
            .get_collision_shape()
            .as_ref()
            .unwrap()
            .borrow()
            .get_aabb(col_obj.get_world_transform());

        let contact_threshold = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);
        min_aabb -= contact_threshold;
        max_aabb += contact_threshold;

        if self.dispatcher_info.use_continuous
            && col_obj.internal_type == CollisionObjectTypes::RigidBody as i32
            && !col_obj.is_static_or_kinematic_object()
        {
            let (mut min_aabb_2, mut max_aabb_2) = col_obj
                .get_collision_shape()
                .as_ref()
                .unwrap()
                .borrow()
                .get_aabb(&col_obj.interpolation_world_transform);
            min_aabb_2 -= contact_threshold;
            max_aabb_2 += contact_threshold;

            min_aabb = min_aabb.min(min_aabb_2);
            max_aabb = max_aabb.max(max_aabb_2);
        }

        if col_obj.is_static_object() || (max_aabb - min_aabb).length_squared() < 1e12 {
            self.broadphase_pair_cache.set_aabb(
                col_obj.get_broadphase_handle().unwrap(),
                min_aabb,
                max_aabb,
            );
        } else {
            unreachable!()
        }
    }

    fn update_aabbs(&mut self) {
        for i in 0..self.collision_objects.len() {
            let col_obj = &self.collision_objects[i];
            debug_assert!(col_obj.borrow().get_world_array_index() as usize == i);

            if self.force_update_all_aabbs || col_obj.borrow().is_active() {
                self.update_single_aabb(i);
            }
        }
    }

    pub fn perform_discrete_collision_detection<T: ContactAddedCallback>(
        &mut self,
        contact_added_callback: &mut T,
    ) {
        self.update_aabbs();

        self.broadphase_pair_cache.calculate_overlapping_pairs();
        self.dispatcher1
            .dispatch_all_collision_pairs(&mut self.broadphase_pair_cache, contact_added_callback);
    }

    fn ray_test_single<T: RayResultCallback>(
        &self,
        ray_from: Vec3A,
        ray_to: Vec3A,
        collision_object: &Rc<RefCell<CollisionObject>>,
        result_callback: &mut T,
    ) {
        let co = collision_object.borrow();

        let world_to_co = co.get_world_transform().inverse();
        let ray_from_local = world_to_co.transform_point3a(ray_from);
        let ray_to_local = world_to_co.transform_point3a(ray_to);

        let shape = co.get_collision_shape().as_ref().unwrap().borrow();
        match &*shape {
            CollisionShapes::Compound(_) => todo!("compound ray test"),
            CollisionShapes::Sphere(_) => todo!("sphere ray test"),
            CollisionShapes::StaticPlane(plane) => {
                let hit_fraction = result_callback.get_base().closest_hit_fraction;
                let mut rcb = BridgeTriangleRaycastCallback::new(
                    ray_from_local,
                    ray_to_local,
                    result_callback,
                    collision_object,
                );
                rcb.hit_fraction = hit_fraction;
                plane.perform_raycast(&mut rcb, ray_from_local, ray_to_local);
            }
            CollisionShapes::TriangleMesh(mesh) => {
                let hit_fraction = result_callback.get_base().closest_hit_fraction;
                let mut rcb = BridgeTriangleRaycastCallback::new(
                    ray_from_local,
                    ray_to_local,
                    result_callback,
                    collision_object,
                );
                rcb.hit_fraction = hit_fraction;
                mesh.perform_raycast(&mut rcb, ray_from_local, ray_to_local);
            }
        }
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
