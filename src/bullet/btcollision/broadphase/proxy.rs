use std::{cell::RefCell, rc::Rc};

use crate::{bullet::btdynamics::dynamics::rigid_body::RigidBody, custom::geometry::Aabb};

#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub enum BroadphaseNativeTypes {
    // polyhedral convex shapes
    BoxShapeProxytype,
    TriangleShapeProxytype,
    TetrahedralShapeProxytype,
    ConvexTrianglemeshShapeProxytype,
    ConvexHullShapeProxytype,
    ConvexPointCloudShapeProxytype,
    CustomPolyhedralShapeType,
    //implicit convex shapes
    ImplicitConvexShapesStartHere,
    SphereShapeProxytype,
    MultiSphereShapeProxytype,
    CapsuleShapeProxytype,
    ConeShapeProxytype,
    ConvexShapeProxytype,
    CylinderShapeProxytype,
    UniformScalingShapeProxytype,
    MinkowskiSumShapeProxytype,
    MinkowskiDifferenceShapeProxytype,
    Box2dShapeProxytype,
    Convex2dShapeProxytype,
    CustomConvexShapeType,
    //concave shapes
    ConcaveShapesStartHere,
    //keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
    TriangleMeshShapeProxytype,
    ScaledTriangleMeshShapeProxytype,
    ///used for demo integration FAST/Swift collision library and Bullet
    FastConcaveMeshProxytype,
    //terrain
    TerrainShapeProxytype,
    ///Used for GIMPACT Trimesh integration
    GimpactShapeProxytype,
    ///Multimaterial mesh
    MultimaterialTriangleMeshProxytype,

    EmptyShapeProxytype,
    StaticPlaneProxytype,
    CustomConcaveShapeType,
    ConcaveShapesEndHere,

    CompoundShapeProxytype,

    SoftbodyShapeProxytype,
    HffluidShapeProxytype,
    HffluidBuoyantConvexShapeProxytype,
    #[default]
    InvalidShapeProxytype,

    MaxBroadphaseCollisionTypes,
}

impl BroadphaseNativeTypes {
    pub const SDF_SHAPE_PROXYTYPE: Self = Self::CustomConcaveShapeType;
}

pub enum CollisionFilterGroups {
    DefaultFilter = 1,
    StaticFilter = 2,
    KinematicFilter = 4,
    DebrisFilter = 8,
    SensorTrigger = 16,
    CharacterFilter = 32,
    AllFilter = -1, //all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger
}

pub struct BroadphaseProxy {
    aabb: Aabb,
    client_object: Rc<RefCell<RigidBody>>,
    collision_filter_group: i32,
    collision_filter_mask: i32,
}

impl BroadphaseProxy {
    pub fn new(
        aabb: Aabb,
        client_object: Rc<RefCell<RigidBody>>,
        collision_filter_group: i32,
        collision_filter_mask: i32,
    ) -> Self {
        Self {
            collision_filter_group,
            collision_filter_mask,
            aabb,
            client_object,
        }
    }

    pub fn default_with(client_object: Rc<RefCell<RigidBody>>) -> Self {
        Self {
            aabb: Aabb::default(),
            client_object,
            collision_filter_group: CollisionFilterGroups::DefaultFilter as i32,
            collision_filter_mask: CollisionFilterGroups::AllFilter as i32,
        }
    }
}
