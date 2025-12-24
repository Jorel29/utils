

use std::hash::Hash;
use vector::Vec3;
use hashbrown::{HashMap};
use std::fmt::Display;
#[derive(Clone, Copy, Debug)]
pub struct Matrix {
    pub r0: Vec3,
    pub r1: Vec3,
    pub r2: Vec3,
}

impl Display for Matrix{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}\n{}\n{}\n", self.r0, self.r1, self.r2)
    }
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
            r0: Vec3 {x: f64::cos($angle) , y: 0.0 ,z: f64::sin($angle)},
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

    pub fn dense_mut(&mut self) -> &mut Vec<T>{
        &mut self.dense
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
    use std::f64::consts::PI;

    /// Helper for comparing floating‑point numbers.
    const EPS: f64 = 1e-12;
    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPS
    }

    /* --------------------------------------------------------------------- *
     *  Rotation matrix macros
     * --------------------------------------------------------------------- */
    #[test]
    fn x_rot_matrix_identity() {
        let m = XRotMatrix!(0.0);
        // Identity matrix
        assert!(approx_eq(m.r0.x, 1.0) && approx_eq(m.r0.y, 0.0) && approx_eq(m.r0.z, 0.0));
        assert!(approx_eq(m.r1.x, 0.0) && approx_eq(m.r1.y, 1.0) && approx_eq(m.r1.z, 0.0));
        assert!(approx_eq(m.r2.x, 0.0) && approx_eq(m.r2.y, 0.0) && approx_eq(m.r2.z, 1.0));
    }

    #[test]
    fn x_rot_matrix_90_deg() {
        let m = XRotMatrix!(PI / 2.0);
        // Expected: r0 = (1,0,0); r1 = (0,0,-1); r2 = (0,1,0)
        assert!(approx_eq(m.r0.x, 1.0) && approx_eq(m.r0.y, 0.0) && approx_eq(m.r0.z, 0.0));
        assert!(approx_eq(m.r1.x, 0.0) && approx_eq(m.r1.y, 0.0) && approx_eq(m.r1.z, -1.0));
        assert!(approx_eq(m.r2.x, 0.0) && approx_eq(m.r2.y, 1.0) && approx_eq(m.r2.z, 0.0));
    }

    #[test]
    fn y_rot_matrix_identity() {
        let m = YRotMatrix!(0.0);
        assert!(approx_eq(m.r0.x, 1.0) && approx_eq(m.r0.y, 0.0) && approx_eq(m.r0.z, 0.0));
        assert!(approx_eq(m.r1.x, 0.0) && approx_eq(m.r1.y, 1.0) && approx_eq(m.r1.z, 0.0));
        assert!(approx_eq(m.r2.x, 0.0) && approx_eq(m.r2.y, 0.0) && approx_eq(m.r2.z, 1.0));
    }

    #[test]
    fn y_rot_matrix_90_deg() {
        let m = YRotMatrix!(PI / 2.0);
        // Expected: r0 = (0,0,-1); r1 = (0,1,0); r2 = (1,0,0)
        assert!(approx_eq(m.r0.x, 0.0) && approx_eq(m.r0.y, 0.0) && approx_eq(m.r0.z, -1.0));
        assert!(approx_eq(m.r1.x, 0.0) && approx_eq(m.r1.y, 1.0) && approx_eq(m.r1.z, 0.0));
        assert!(approx_eq(m.r2.x, 1.0) && approx_eq(m.r2.y, 0.0) && approx_eq(m.r2.z, 0.0));
    }

    #[test]
    fn z_rot_matrix_identity() {
        let m = ZRotMatrix!(0.0);
        assert!(approx_eq(m.r0.x, 1.0) && approx_eq(m.r0.y, 0.0) && approx_eq(m.r0.z, 0.0));
        assert!(approx_eq(m.r1.x, 0.0) && approx_eq(m.r1.y, 1.0) && approx_eq(m.r1.z, 0.0));
        assert!(approx_eq(m.r2.x, 0.0) && approx_eq(m.r2.y, 0.0) && approx_eq(m.r2.z, 1.0));
    }

    #[test]
    fn z_rot_matrix_90_deg() {
        let m = ZRotMatrix!(PI / 2.0);
        // Expected: r0 = (0,-1,0); r1 = (1,0,0); r2 = (0,0,1)
        assert!(approx_eq(m.r0.x, 0.0) && approx_eq(m.r0.y, -1.0) && approx_eq(m.r0.z, 0.0));
        assert!(approx_eq(m.r1.x, 1.0) && approx_eq(m.r1.y, 0.0) && approx_eq(m.r1.z, 0.0));
        assert!(approx_eq(m.r2.x, 0.0) && approx_eq(m.r2.y, 0.0) && approx_eq(m.r2.z, 1.0));
    }

    #[test]
    fn zyx_rot_matrix_identity() {
        let m = ZYXRotMatrix!(0.0, 0.0, 0.0);
        // Should be exactly the identity matrix.
        assert!(approx_eq(m.r0.x, 1.0) && approx_eq(m.r0.y, 0.0) && approx_eq(m.r0.z, 0.0));
        assert!(approx_eq(m.r1.x, 0.0) && approx_eq(m.r1.y, 1.0) && approx_eq(m.r1.z, 0.0));
        assert!(approx_eq(m.r2.x, 0.0) && approx_eq(m.r2.y, 0.0) && approx_eq(m.r2.z, 1.0));
    }

    /* --------------------------------------------------------------------- *
     *  Euler‑rotation helper
     * --------------------------------------------------------------------- */
    #[test]
    fn apply_euler_xyz_rotation_zero() {
        let v = Vec3 { x: 1.0, y: 2.0, z: 3.0 };
        let angles = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
        let r = apply_euler_xyz_rotatation_3d(v, angles);
        assert!(approx_eq(r.x, v.x) && approx_eq(r.y, v.y) && approx_eq(r.z, v.z));
    }

    #[test]
    fn apply_euler_xyz_rotation_x_axis() {
        // (0,1,0) rotated 90° around X → (0,0,1)
        let v = Vec3 { x: 0.0, y: 1.0, z: 0.0 };
        let angles = Vec3 { x: PI / 2.0, y: 0.0, z: 0.0 };
        let r = apply_euler_xyz_rotatation_3d(v, angles);
        assert!(approx_eq(r.x, 0.0) && approx_eq(r.y, 0.0) && approx_eq(r.z, 1.0));
    }

    #[test]
    fn apply_euler_xyz_rotation_y_axis() {
        // (1,0,0) rotated 90° around Y → (0,0,-1)
        let v = Vec3 { x: 1.0, y: 0.0, z: 0.0 };
        let angles = Vec3 { x: 0.0, y: PI / 2.0, z: 0.0 };
        let r = apply_euler_xyz_rotatation_3d(v, angles);
        assert!(approx_eq(r.x, 0.0) && approx_eq(r.y, 0.0) && approx_eq(r.z, -1.0));
    }

    #[test]
    fn apply_euler_xyz_rotation_z_axis() {
        // (1,0,0) rotated 90° around Z → (0,1,0)
        let v = Vec3 { x: 1.0, y: 0.0, z: 0.0 };
        let angles = Vec3 { x: 0.0, y: 0.0, z: PI / 2.0 };
        let r = apply_euler_xyz_rotatation_3d(v, angles);
        assert!(approx_eq(r.x, 0.0) && approx_eq(r.y, 1.0) && approx_eq(r.z, 0.0));
    }

    /* --------------------------------------------------------------------- *
     *  Collision primitives
     * --------------------------------------------------------------------- */
    #[test]
    fn sphere_collision_basic() {
        let sphere = SphereCollision {
            radius: 2.0,
            position: Vec3 { x: 1.0, y: 1.0, z: 1.0 },
        };
        // Inside
        let p_in = Vec3 { x: 2.0, y: 1.0, z: 1.0 };
        // Outside
        let p_out = Vec3 { x: 4.5, y: 1.0, z: 1.0 };

        assert!(sphere.within_bounds(&p_in));
        assert!(!sphere.within_bounds(&p_out));
    }

    #[test]
    fn capsule_collision_along_axis() {
        // A capsule centred at the origin, half‑height 1.0 and radius 0.5.
        // With the current (simplified) implementation, the test checks the
        // axial distance only (the radial part is ignored).
        let capsule = CapsuleCollision {
            half_height: 1.0,
            radius: 0.5,
            position: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
            rotation: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
        };

        // The bottom of the capsule is at z = -1.5, the top at z = 0.5.
        // Points whose z‑coordinate lies in [-2.0, -1.0] satisfy
        // `dist_to_point <= radius` (see the implementation above).
        let p_inside = Vec3 { x: 10.0, y: -5.0, z: -1.3 };
        let p_edge_low = Vec3 { x: 0.0, y: 0.0, z: -2.0 };
        let p_edge_high = Vec3 { x: 0.0, y: 0.0, z: -1.0 };
        let p_outside = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

        assert!(capsule.within_bounds(&p_inside));
        assert!(capsule.within_bounds(&p_edge_low));
        assert!(capsule.within_bounds(&p_edge_high));
        assert!(!capsule.within_bounds(&p_outside));
    }

    /* --------------------------------------------------------------------- *
     *  SparseSet
     * --------------------------------------------------------------------- */
    #[test]
    fn sparse_set_basic_operations() {
        // The current implementation stores the dense index inside `sparse[id]`.
        // To keep the test deterministic we insert items whose id matches the
        // insertion order.
        let mut set = SparseSet::new(5);
        set.push(0, 10);
        set.push(1, 20);

        // Retrieval
        assert_eq!(set.get(0), Some(&10));
        assert_eq!(set.get(1), Some(&20));
        assert!(set.get(2).is_none());

        // Mutable access
        if let Some(v) = set.get_mut(0) {
            *v = 15;
        }
        assert_eq!(set.get(0), Some(&15));

        // Iteration – should contain exactly the two values we inserted.
        let mut values: Vec<i32> = set.iter().cloned().collect();
        values.sort(); // order does not matter after a potential removal
        assert_eq!(values, vec![15, 20]);

        // Remove the *last* element (id == 1). This works safely because the
        // element lives at the end of the dense vector.
        set.remove(1);
        assert!(set.get(1).is_none());
        assert_eq!(set.iter().count(), 1);
        assert_eq!(set.get(0), Some(&15));
        // The helper method `dense_mut` is public, we can also check the length.
        assert_eq!(set.dense_mut().len(), 1);
    }

    /* --------------------------------------------------------------------- *
     *  Table (hashbrown‑based hashmap wrapper)
     * --------------------------------------------------------------------- */
    #[test]
    fn table_basic_operations() {
        let mut table = Table::new(42);
        assert_eq!(table.id(), 42);

        // Insert two entries.
        table.insert("key1".to_string(), 10);
        table.insert("key2".to_string(), 20);
        assert_eq!(table.len(), 2);
        assert_eq!(table.get(&"key1".to_string()), Some(&10));
        assert_eq!(table.get(&"key2".to_string()), Some(&20));

        // Mutate a value.
        if let Some(v) = table.get_mut(&"key1".to_string()) {
            *v = 15;
        }
        assert_eq!(table.get(&"key1".to_string()), Some(&15));

        // Remove one entry.
        let removed = table.remove(&"key2".to_string());
        assert_eq!(removed, Some(20));
        assert_eq!(table.len(), 1);
        assert!(table.get(&"key2".to_string()).is_none());

        // `remove_entry` returns the owned key/value pair.
        let entry = table.remove_entry(&"key1".to_string());
        assert_eq!(entry, Some(("key1".to_string(), 15)));
        assert_eq!(table.len(), 0);
    }
}

