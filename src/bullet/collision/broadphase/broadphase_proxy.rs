use crate::bullet::linear_math::aabb_util_2::Aabb;

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
    Default = 1,
    Static = (1 << 1),
    All = -1,
}

#[derive(Clone, Default)]
pub struct BroadphaseProxy {
    /// The index of the client `CollisionObject` in `CollisionWorld`
    pub client_object_idx: Option<usize>,
    pub collision_filter_group: u8,
    pub collision_filter_mask: u8,
    pub unique_id: usize,
    pub aabb: Aabb,
}

pub struct BroadphasePair {
    pub proxy0: usize,
    pub proxy1: usize,
}

pub trait BroadphaseAabbCallback {
    fn process(&mut self, proxy: &BroadphaseProxy) -> bool;
}
