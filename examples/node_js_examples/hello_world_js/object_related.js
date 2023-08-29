/*
Way to get all keys: Object.keys(obj)
*/
var obj = {
    "a": 1
}

console.log(Object.keys(obj));

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
