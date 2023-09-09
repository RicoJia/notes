const test_map = () => {
    const m = new Map<string, string>([
        ["a", "a_val"],
        ["b", "b_val"],
    ]);

    /*
    Array, Maps, Sets have forEach, string doesn't
    Datastructures with Symbol.iterator method have forEach
    */
    m.forEach((v, k) => {
       console.log("map v: ", v, "map k: ", k); 
    })
}
test_map();