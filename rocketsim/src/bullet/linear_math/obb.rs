use glam::{Mat3A, Vec3A};

pub struct Obb {
    pub center: Vec3A,
    pub axis: Mat3A,
    pub extent: Vec3A,
}

impl Obb {
    pub const fn new(center: Vec3A, axis: Mat3A, extent: Vec3A) -> Self {
        Self {
            center,
            axis,
            extent,
        }
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
}
