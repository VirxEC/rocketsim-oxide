use crate::{
    bullet::{
        btcollision::broadphase::proxy::BroadphaseNativeTypes, linear_math::transform::Transform,
    },
    custom::geometry::Aabb,
    BvhTriangleMeshShape,
};

#[derive(Default)]
pub enum CollisionShape {
    // SPHERE_SHAPE_PROXYTYPE
    // STATIC_PLANE_PROXYTYPE
    TriangleMeshShapeProxytype(BvhTriangleMeshShape),
    #[default]
    None,
}

impl CollisionShape {
    #[inline]
    pub fn is_some(&self) -> bool {
        !self.is_none()
    }

    #[inline]
    pub fn is_none(&self) -> bool {
        matches!(self, Self::None)
    }

    #[inline]
    pub fn get_shape_type(&self) -> BroadphaseNativeTypes {
        match self {
            Self::TriangleMeshShapeProxytype(_) => {
                BroadphaseNativeTypes::TriangleMeshShapeProxytype
            }
            Self::None => BroadphaseNativeTypes::InvalidShapeProxytype,
        }
    }

    #[inline]
    pub fn get_aabb(&self, trans: Transform) -> Aabb {
        match self {
            Self::TriangleMeshShapeProxytype(shape) => shape.get_aabb(trans),
            Self::None => Aabb::default(),
        }
    }
}

impl From<BvhTriangleMeshShape> for CollisionShape {
    #[inline]
    fn from(shape: BvhTriangleMeshShape) -> Self {
        Self::TriangleMeshShapeProxytype(shape)
    }
}
