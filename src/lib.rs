

use std::hash::Hash;
use vector::Vec3;
use hashbrown::{HashMap};
#[derive(Clone, Copy)]
pub struct Matrix {
    pub r0: Vec3,
    pub r1: Vec3,
    pub r2: Vec3,
}

#[macro_export]
macro_rules! XRotMatrix {
    ($angle:expr) => {
        Matrix{
            r0: Vec3 {x: 1.0 , y: 0.0 ,z: 0.0},
            r1: Vec3 {x: 0.0 , y: f64::cos($angle), z: -1.0*f64::sin($angle)},
            r2: Vec3 {x: 0.0, y: f64::sin($angle), z: f64::cos($angle)},  
        }
    };
}

#[macro_export]
macro_rules! YRotMatrix {
    ($angle:expr) => {
        Matrix{
            r0: Vec3 {x: f64::cos($angle) , y: 0.0 ,z: -f64::sin($angle)},
            r1: Vec3 {x: 0.0 , y: 1.0, z: 0.0},
            r2: Vec3 {x: -1.0*f64::sin($angle), y: 0.0, z: f64::cos($angle)},  
        }
    };
}

#[macro_export]
macro_rules! ZRotMatrix {
    ($angle:expr) => {
        Matrix{
            r0: Vec3 {x: f64::cos($angle) , y: -1.0*f64::sin($angle) , z: 0.0},
            r1: Vec3 {x: f64::sin($angle) , y: f64::cos($angle), z: 0.0},
            r2: Vec3 {x: 0.0 , y: 0.0, z: 1.0},
        }
    };
}

#[macro_export]
macro_rules! ZYXRotMatrix {
    ($yaw:expr, $pitch:expr, $roll:expr) => {
       Matrix{
        r0: Vec3{ x: f64::cos($yaw)*f64::cos($pitch),
                    y: f64::cos($yaw)*f64::sin($pitch)*f64::sin($roll) - f64::sin($yaw)*f64::cos($roll),
                    z: f64::cos($yaw)*f64::sin($pitch)*f64::cos($roll) + f64::sin($yaw)*f64::sin($roll), },
        r1: Vec3{ x: f64::sin($yaw)*f64::cos($pitch),
                    y: f64::sin($yaw)*f64::sin($pitch)*f64::sin($roll) + f64::cos($yaw)*f64::cos($roll),
                    z: f64::sin($yaw)*f64::sin($pitch)*f64::cos($roll) - f64::cos($yaw)*f64::sin($roll),},
        r2: Vec3{ x: -1.0*f64::sin($pitch),
                    y: f64::cos($pitch)*f64::sin($roll),
                    z: f64::cos($pitch)*f64::cos($roll),},
       } 
    };
}

pub fn apply_euler_xyz_rotatation_3d(vec: Vec3, angle: Vec3) -> Vec3{

    
    let x_rot = XRotMatrix!(angle.x);
    let y_rot = YRotMatrix!(angle.y);
    let z_rot = ZRotMatrix!(angle.z);

    //apply x axis rotation
    let x_prime = Vec3::dot(&x_rot.r0, &vec);
    let y_prime = Vec3::dot(&x_rot.r1, &vec);
    let z_prime = Vec3::dot(&x_rot.r2, &vec);

    let vec_prime = Vec3{x: x_prime, y: y_prime, z: z_prime};

    //apply y axis rotation
    let x_2prime = Vec3::dot(&y_rot.r0, &vec_prime);
    let y_2prime = Vec3::dot(&y_rot.r1, &vec_prime);
    let z_2prime = Vec3::dot(&y_rot.r2, &vec_prime);

    let vec_2prime = Vec3{x: x_2prime, y: y_2prime, z: z_2prime};

    //apply z axis rotation
    let x_3prime = Vec3::dot(&z_rot.r0, &vec_2prime);
    let y_3prime = Vec3::dot(&z_rot.r1, &vec_2prime);
    let z_3prime = Vec3::dot(&z_rot.r2, &vec_2prime);

    Vec3 { x: x_3prime, y: y_3prime, z: z_3prime }
}

#[derive(Debug)]
pub struct CapsuleCollision{
    pub half_height: f64,
    pub radius: f64,
    pub position: Vec3,
    pub rotation: Vec3,
}

impl CapsuleCollision {

    pub fn new()->CapsuleCollision{
        CapsuleCollision { half_height: 0.0, radius: 0.0, position: Vec3 { x: 0.0, y: 0.0, z: 0.0 }, rotation: Vec3 { x: 0.0, y: 0.0, z: 0.0 } }
    }

    // Current implementation assumes capsule is not rotatable and does not use line formula and radius check
    pub fn within_bounds(&self, pos: &Vec3) -> bool{
        
        

        
        
        let cyl_top = Vec3 {
            x: self.position.x,
            y: self.position.y,
            z: self.position.z + self.half_height-self.radius,
        };

        let cyl_bot =  Vec3 {
            x: self.position.x,
            y: self.position.y,
            z: self.position.z - self.half_height-self.radius,
        };

        let ap = *pos - cyl_bot;

        let ab = cyl_top - cyl_bot;

        let point_on_line = *pos + (ab * Vec3::dot(&ap, &ab)/ Vec3::dot(&ab, &ab));

        let dist_to_point = Vec3::distance(&point_on_line, &pos);

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
    pub position: Vec3,
}

impl SphereCollision{

    pub fn new() -> SphereCollision{
        SphereCollision { radius: 0.0 , position: Vec3 { x: 0.0, y: 0.0, z: 0.0 } }
    }

    pub fn within_bounds(&self, pos: &Vec3) -> bool{
        
        if f64::abs(Vec3::distance(&self.position, &pos)) <= self.radius{
            true
        }else{
            false
        }
    }
}
#[derive(Debug)]
pub struct SparseSet<T>{
    dense: Vec<T>,
    sparse: Vec<Option<usize>>
}

impl <T> SparseSet<T>{
    pub fn new(max_capacity:usize)-> Self{
        Self { 
            dense: Vec::new(),
            sparse: vec![None; max_capacity]
        }
    }

    pub fn push(&mut self, id:usize, item: T){
        self.dense.push(item);
        self.sparse[id]= Some(id);
    }

    pub fn remove(&mut self, id:usize ){
        if let Some(index) = self.sparse[id] {
            self.dense.swap_remove(index);
        }
    }

    pub fn get(&self, id:usize)-> Option<&T>{
        if let Some(ind) = self.sparse[id]{
            self.dense.get(ind)
        }else{
            None
        }
    }

    pub fn get_mut(&mut self, id:usize) -> Option<&mut T>{
        if let Some(ind) = self.sparse[id]{
            self.dense.get_mut(ind)
        }else{
            None
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &T>{
        self.dense.iter()
    }

    pub fn iter_mut(&mut self)-> impl Iterator<Item = &mut T>{
        self.dense.iter_mut()
    }
}

// Wrapper for Hashmap
pub struct Table<K, V>
where 
    K: Hash + Eq
{
    id: u32,
    table: HashMap<K, V>
}

impl <K, V> Table<K, V>
where 
    K: Hash + Eq
{
    pub fn new(table_id: u32)->Self{
        Self { 
            id: table_id,
            table: HashMap::new() 
        }
    }

    pub fn insert(&mut self, k: K, v: V) -> Option<V>{
        self.table.insert(k, v)
    }

    pub fn remove(&mut self, k: &K) -> Option<V>{
        self.table.remove(k)
    }

    pub fn remove_entry(&mut self, k: &K) -> Option<(K, V)>{
        self.table.remove_entry(k)
    }

    pub fn len(&self) -> usize {
        self.table.len()
    }

    pub fn get(&self, k: &K)-> Option<&V>{
        self.table.get(k)
    }

    pub fn get_key_value(&self, k: &K)-> Option<(&K,&V)>{
        self.table.get_key_value(k)
    }

    pub fn get_mut(&mut self, k: &K)->Option<&mut V>{
        self.table.get_mut(k)
    }

    pub fn get_key_value_mut(&mut self, k: &K) -> Option<(&K, &mut V)>{
        self.table.get_key_value_mut(k)
    }

    pub fn id(&self) -> u32{
        self.id
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
            position: Vec3 { x:0.0, y: 0.0, z: 0.0 },
            rotation: Vec3 { x: 0.0, y: 0.0, z: 0.0 }
        };

        let position = Vec3{
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

        let position = Vec3{
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };

        let result = sphere.within_bounds(&position);
        assert!(result);

    }
}
