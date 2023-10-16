// You can't do 1=='11, as TS is a strongly typed language
var a = 1;
// Type script doesn't support 1=='1'
console.log(Number(a), typeof null, 1===1, 1==1);

/* 
    There's == in TS (allowing type coersion, like true == 1)
    but I think it's basically useless, because true == 1 will 
    yield an error anyways. 
*/
function test_equality(){
    // console.log(true === 1);
    // console.log(true == 1);
}
test_equality();

function test_array(){
    // const only makes the variable name bound to the object. But you can still modify its properties
    const array = [1,2,3,4, 5, 5];
    for (let i = 0; i < array.length/2; i++) {
        [array[i], array[array.length - 1 - i]] = [array[array.length - 1 - i], array[i]];
    }
    var tempArray = [...array];
    console.log("const array is const reference to an array, but its elements can still be changed:  ", array);

    let const_new_arr: ReadonlyArray<number> = [1,2,3,4,5,5];
    
    // Shallow read-only object. (deeper properties can still be modified)
    const const_arr = Object.freeze([1,2,3]);
    // this won't throw an error, but will have no effect in JS; In TS, it will throw an error
    // const_arr[0] = 2;
    console.log("finding index, if not existent, -1: ", const_new_arr.findIndex(x => x === 6));
    console.log("finding first index, if existent: ", const_new_arr.findIndex(x => x === 5));
}

test_array();

const test_generics = () => {
    // generics, a feature only in TS. Gives you more idea of what is being passed in
    function getFirstElement<T>(arr: T[]): T {
        return arr[0];
    }
    let arr = [1,2,3];
    console.log(getFirstElement(arr));
};

// try catch finally will be executed,after an exception is thrown
const test_throw = () => {
    try{
        throw new Error("This is an error");
    }
    catch(err){
        throw err;
    }
    finally{
        console.log("finally");
    }
}
// test_throw();

const test_record = () => {
    type LoggingInfoType = Record<string, number>;
    const loggingInfo: LoggingInfoType = {
        ipAddress: 1,
        port: 1
    };
    console.log('loggingInfo: ', loggingInfo);
}
test_record();
