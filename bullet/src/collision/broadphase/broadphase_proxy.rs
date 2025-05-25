use crate::collision::dispatch::collision_object::CollisionObject;
use glam::Vec3A;
use std::{cell::RefCell, rc::Rc};

#[allow(dead_code)]
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq, PartialOrd, Ord)]
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
    Box2DShapeProxytype,
    Convex2DShapeProxytype,
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
    // SdfShapeProxytype = CustomConcaveShapeType,
    ConcaveShapesEndHere,

    CompoundShapeProxytype,

    SoftbodyShapeProxytype,
    HfFluidShapeProxytype,
    HfFluidBuoyantConvexShapeProxytype,
    #[default]
    InvalidShapeProxytype,

    MaxBroadphaseCollisionTypes,
}

pub enum CollisionFilterGroups {
    DefaultFilter = 1,
    StaticFilter = 2,
    KinematicFilter = 4,
    DebrisFilter = 8,
    SensorTrigger = 16,
    CharacterFilter = 32,
    AllFilter = -1,
}

#[derive(Clone, Default)]
pub struct BroadphaseProxy {
    pub client_object: Rc<RefCell<CollisionObject>>,
    pub collision_filter_group: i32,
    pub collision_filter_mask: i32,
    pub unique_id: i32,
    pub aabb_min: Vec3A,
    pub aabb_max: Vec3A,
}

pub struct BroadphasePair {
    proxy0: BroadphaseProxy,
    proxy1: BroadphaseProxy,
    // mutable btCollisionAlgorithm* m_algorithm;
}
