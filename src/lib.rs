use vector::Vector;
#[derive(Clone, Copy)]
struct RotMatrix {
    c0: Vector,
    c1: Vector,
    c2: Vector,
}

#[macro_export]
macro_rules! XRotMatrix {
    ($angle:expr) => {
        RotMatrix{
            c0: Vector {x: 1.0 , y: 0.0 ,z: 0.0},
            c1: Vector {x: 0.0 , y: f64::cos($angle), z: f64::sin($angle)},
            c2: Vector {x: 0.0, y: -1.0*f64::sin($angle), z: f64::cos($angle)},  
        }
    };
}

#[macro_export]
macro_rules! YRotMatrix {
    ($angle:expr) => {
        RotMatrix{
            c0: Vector {x: f64::cos($angle) , y: 0.0 ,z: -1.0*f64::sin($angle)},
            c1: Vector {x: 0.0 , y: 1.0, z: 0.0},
            c2: Vector {x: f64:sin($angle), y: 0.0, z: f64::cos($angle)},  
        }
    };
}

#[macro_export]
macro_rules! ZRotMatrix {
    ($angle:expr) => {
        RotMatrix{
            c0: Vector {x: f64::cos($angle) , y: f64::sin($angle) , z: 0.0},
            c1: Vector {x: -1.0*f64::sin($angle) , y: f64::cos($angle), z: 0.0},
            c2: Vector {x: 0.0 , y: 0.0, z: 1.0},
        }
    };
}

pub fn apply_euler_xyz_rotatation_3d(vec: Vector, angle: Vector) -> Vector{

    
    let x_rot = XRotMatrix!(angle.x);
    let y_rot = YRotMatrix!(angle.y);
    let z_rot = ZRotMatrix!(angle.z);


    vec
}


pub struct CapsuleCollision{
    pub half_height: f64,
    pub radius: f64,
    pub position: Vector,
    pub rotation: Vector,
}

impl CapsuleCollision {

    pub fn new()->CapsuleCollision{
        CapsuleCollision { half_height: 0.0, radius: 0.0, position: Vector { x: 0.0, y: 0.0, z: 0.0 }, rotation: Vector { x: 0.0, y: 0.0, z: 0.0 } }
    }

    // Current implementation assumes capsule is not rotatable and does not use line formula and radius check
    pub fn within_bounds(&self, pos: &Vector) -> bool{
        
        
        let rot_x = self.rotation.x;
        let rot_y = self.rotation.y;
        let rot_z = self.rotation.z;

        
        
        let cyl_top = Vector {
            x: self.position.x,
            y: self.position.y,
            z: self.position.z + self.half_height-self.radius,
        };

        let cyl_bot =  Vector {
            x: self.position.x,
            y: self.position.y,
            z: self.position.z - self.half_height-self.radius,
        };

        let ap = *pos - cyl_bot;

        let ab = cyl_top - cyl_bot;

        let point_on_line = *pos + (ab * Vector::dot(&ap, &ab)/ Vector::dot(&ab, &ab));

        let dist_to_point = Vector::distance(&point_on_line, &pos);

        if dist_to_point <= self.radius {
            true
        }else{
            false
        }
    }

}


struct SphereCollision {
    pub radius: f64,
    pub position: Vector,
}

impl SphereCollision{

    pub fn new() -> SphereCollision{
        SphereCollision { radius: 0.0 , position: Vector { x: 0.0, y: 0.0, z: 0.0 } }
    }

    pub fn within_bounds(&self, pos: &Vector) -> bool{
        
        if f64::abs(Vector::distance(&self.position, &pos)) <= self.radius{
            true
        }else{
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capsule_check_bounds() {
        let capsule = CapsuleCollision{
            half_height: 5.0,
            radius: 2.0,
            position: Vector { x:0.0, y: 0.0, z: 0.0 },
            rotation: Vector { x: 0.0, y: 0.0, z: 0.0 }
        };

        let position = Vector{
            x:1.0,
            y:1.0,
            z:1.0,
        };
        let result = capsule.within_bounds(&position);
        assert!(result, "Check if point is within bounds");
    }

    #[test]
    fn sphere_check_bounds(){
        let mut sphere = SphereCollision::new();

        sphere.radius = 2.0;

        let position = Vector{
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };

        let result = sphere.within_bounds(&position);
        assert!(result);

    }
}
