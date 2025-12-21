use glam::{Mat3A, Vec3A};

pub struct Obb {
    pub center: Vec3A,
    pub axis: Mat3A,
    pub extent: Vec3A,
    world_extents: Mat3A,
}

impl Obb {
    pub fn new(center: Vec3A, axis: Mat3A, extent: Vec3A) -> Self {
        let world_extents = Mat3A::from_cols(
            axis.x_axis * extent.x,
            axis.y_axis * extent.y,
            axis.z_axis * extent.z,
        );

        Self {
            center,
            axis,
            extent,
            world_extents,
        }
    }

    /// Project an OBB onto an axis, returning the “radius” (half-projection length)
    pub fn project_obb_radius(&self, axis: Vec3A) -> f32 {
        axis.dot(self.world_extents.x_axis).abs()
            + axis.dot(self.world_extents.y_axis).abs()
            + axis.dot(self.world_extents.z_axis).abs()
    }

    pub fn closest_point_on_obb_edge(&self, point: Vec3A) -> Vec3A {
        let d = point - self.center;
        let mut q = self.center;

        for (i, axis) in [self.axis.x_axis, self.axis.y_axis, self.axis.z_axis]
            .into_iter()
            .enumerate()
        {
            let dist = d.dot(axis);
            let clamped = dist.clamp(-self.extent[i], self.extent[i]);
            q += clamped * axis;
        }

        q
    }

    /// Returns the 4 corners of the face on the `axis_idx` (0 = X, 1 = Y, 2 = Z)
    /// and `side_sign` = +1 or -1
    pub fn get_face_verts(&self, face_axis_idx: usize, side_sign: f32) -> [Vec3A; 4] {
        let axis = [self.axis.x_axis, self.axis.y_axis, self.axis.z_axis];

        let u = axis[(face_axis_idx + 1) % 3];
        let v = axis[(face_axis_idx + 2) % 3];

        let eu = self.extent[(face_axis_idx + 1) % 3];
        let ev = self.extent[(face_axis_idx + 2) % 3];

        let ueu = u * eu;
        let vev = v * ev;

        let center = self.center + axis[face_axis_idx] * self.extent[face_axis_idx] * side_sign;

        // CCW ordering
        [
            center + ueu + vev,
            center + ueu - vev,
            center - ueu - vev,
            center - ueu + vev,
        ]
    }

    /// Returns the four corners of the face on `obb` whose face normal is closest to `normal`
    pub fn find_face_verts(&self, normal: Vec3A) -> [Vec3A; 4] {
        let axes = [self.axis.x_axis, self.axis.y_axis, self.axis.z_axis];
        let (face_axis_idx, face_axis) = axes
            .into_iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                normal
                    .dot(*a)
                    .abs()
                    .partial_cmp(&normal.dot(*b).abs())
                    .unwrap()
            })
            .unwrap();

        let side_sign = normal.dot(face_axis).signum();
        self.get_face_verts(face_axis_idx, side_sign)
    }
}
