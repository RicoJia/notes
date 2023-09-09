/*
Objects basics: 
    - using {}
    - if a name is not valid for binding, double quote them. Else no need
    - Way to get all keys: Object.keys(obj)
*/
const test_object_basics = () => {
    var obj = {
        somProperty: "haha",
        "a": 1
    }

    console.log("keys of obj", Object.keys(obj));

}

/**
 * ================ Array ================
 */
// This is an obj
var SomeObj = {
    a: 1,
    b: "hola"
};
var arr = ["1", "2", "3"];
// array is an subtype of object, so you can do
console.log(arr.length, SomeObj.a, typeof(SomeObj), typeof(arr));
typeof arr;

// Convert to each array element
// creates a new array, by iterating through each element in readCoilResult, and converting it to number
arr.map((i) => Number(i)) 
// replace 2 elements start at index 1 with "haha"
arr.splice(1,2,"haha")
// guess what this does ;)
arr.splice(1,2)
console.log("What does array look like?", arr)

const test_filter = () => {
    let arr = [true, false, true, true];
    let new_arr = arr.filter(x => x);
    console.log("test filter, removed falsy elements with filter: ", new_arr);
}
test_filter();