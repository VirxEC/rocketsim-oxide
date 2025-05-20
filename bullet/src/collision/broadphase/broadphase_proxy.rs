#[allow(dead_code)]
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
    InvalidShapeProxytype,

    MaxBroadphaseCollisionTypes,
}
