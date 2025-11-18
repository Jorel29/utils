use vector::Vector;

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
