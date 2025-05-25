pub struct Element {
    id: i32,
    sz: i32,
}

#[derive(Default)]
pub struct UnionFind {
    elements: Vec<Element>,
}
