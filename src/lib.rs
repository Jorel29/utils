use vector::Vector;
#[derive(Clone, Copy)]
pub struct Matrix {
    pub r0: Vector,
    pub r1: Vector,
    pub r2: Vector,
}

#[macro_export]
macro_rules! XRotMatrix {
    ($angle:expr) => {
        Matrix{
            r0: Vector {x: 1.0 , y: 0.0 ,z: 0.0},
            r1: Vector {x: 0.0 , y: f64::cos($angle), z: -1.0*f64::sin($angle)},
            r2: Vector {x: 0.0, y: f64::sin($angle), z: f64::cos($angle)},  
        }
    };
}

#[macro_export]
macro_rules! YRotMatrix {
    ($angle:expr) => {
        Matrix{
            r0: Vector {x: f64::cos($angle) , y: 0.0 ,z: -f64::sin($angle)},
            r1: Vector {x: 0.0 , y: 1.0, z: 0.0},
            r2: Vector {x: -1.0*f64::sin($angle), y: 0.0, z: f64::cos($angle)},  
        }
    };
}

#[macro_export]
macro_rules! ZRotMatrix {
    ($angle:expr) => {
        Matrix{
            r0: Vector {x: f64::cos($angle) , y: -1.0*f64::sin($angle) , z: 0.0},
            r1: Vector {x: f64::sin($angle) , y: f64::cos($angle), z: 0.0},
            r2: Vector {x: 0.0 , y: 0.0, z: 1.0},
        }
    };
}

#[macro_export]
macro_rules! ZYXRotMatrix {
    ($yaw:expr, $pitch:expr, $roll:expr) => {
       Matrix{
        r0: Vector{ x: f64::cos($yaw)*f64::cos($pitch),
                    y: f64::cos($yaw)*f64::sin($pitch)*f64::sin($roll) - f64::sin($yaw)*f64::cos($roll),
                    z: f64::cos($yaw)*f64::sin($pitch)*f64::cos($roll) + f64::sin($yaw)*f64::sin($roll), },
        r1: Vector{ x: f64::sin($yaw)*f64::cos($pitch),
                    y: f64::sin($yaw)*f64::sin($pitch)*f64::sin($roll) + f64::cos($yaw)*f64::cos($roll),
                    z: f64::sin($yaw)*f64::sin($pitch)*f64::cos($roll) - f64::cos($yaw)*f64::sin($roll),},
        r2: Vector{ x: -1.0*f64::sin($pitch),
                    y: f64::cos($pitch)*f64::sin($roll),
                    z: f64::cos($pitch)*f64::cos($roll),},
       } 
    };
}

pub fn apply_euler_xyz_rotatation_3d(vec: Vector, angle: Vector) -> Vector{

    
    let x_rot = XRotMatrix!(angle.x);
    let y_rot = YRotMatrix!(angle.y);
    let z_rot = ZRotMatrix!(angle.z);

    //apply x axis rotation
    let x_prime = Vector::dot(&x_rot.r0, &vec);
    let y_prime = Vector::dot(&x_rot.r1, &vec);
    let z_prime = Vector::dot(&x_rot.r2, &vec);

    let vec_prime = Vector{x: x_prime, y: y_prime, z: z_prime};

    //apply y axis rotation
    let x_2prime = Vector::dot(&y_rot.r0, &vec_prime);
    let y_2prime = Vector::dot(&y_rot.r1, &vec_prime);
    let z_2prime = Vector::dot(&y_rot.r2, &vec_prime);

    let vec_2prime = Vector{x: x_2prime, y: y_2prime, z: z_2prime};

    //apply z axis rotation
    let x_3prime = Vector::dot(&z_rot.r0, &vec_2prime);
    let y_3prime = Vector::dot(&z_rot.r1, &vec_2prime);
    let z_3prime = Vector::dot(&z_rot.r2, &vec_2prime);

    Vector { x: x_3prime, y: y_3prime, z: z_3prime }
}

#[derive(Debug)]
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

#[derive(Debug)]
pub struct SphereCollision {
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
