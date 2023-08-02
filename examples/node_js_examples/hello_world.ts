// You can't do 1=='11, as TS is a strongly typed language
var a = 1;
// Type script doesn't support 1=='1'
console.log(Number(a), typeof null, 1===1, 1==1);

function test_array(){
    // const only makes the variable name bound to the object. But you can still modify its properties
    const array = [1,2,3,4,5];
    for (let i = 0; i < array.length/2; i++) {
        [array[i], array[array.length - 1 - i]] = [array[array.length - 1 - i], array[i]];
    }
    console.log(array);
    var tempArray = [...array];
    
    // Shallow read-only object. (deeper properties can still be modified)
    const const_arr = Object.freeze([1,2,3]);
    // this won't throw an error, but will have no effect in JS; In TS, it will throw an error
    // const_arr[0] = 2;
}

test_array()