use vector::Vector;

pub struct CapsuleCollision{
    half_height: f64,
    radius: f64,
    position: Vector,
    rotation: Vector,
}

impl CapsuleCollision {

    pub fn new()->CapsuleCollision{
        CapsuleCollision { half_height: 0.0, radius: 0.0, position: Vector { x: 0.0, y: 0.0, z: 0.0 }, rotation: Vector { x: 0.0, y: 0.0, z: 0.0 } }
    }

    // Current implementation assumes capsule is not rotatable
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

        if f64::abs(Vector::distance(&cyl_top, &pos)) <= self.radius || f64::abs(Vector::distance(&cyl_bot, &pos)) <= self.radius{
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
    fn check_bounds() {
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
}
