use super::{convex_internal_shape::ConvexInternalShape, convex_polyhedron::ConvexPolyhedron};

pub struct PolyhedralConvexShape {
    pub convex_internal_shape: ConvexInternalShape,
    pub polyhedron: ConvexPolyhedron,
}
