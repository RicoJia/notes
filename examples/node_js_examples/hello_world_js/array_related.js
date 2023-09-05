const test_filter = () => {
    let arr = [true, false, true, true];
    let new_arr = arr.filter(x => x);
    console.log("test filter, removed falsy elements with filter: ", new_arr);
}
test_filter();