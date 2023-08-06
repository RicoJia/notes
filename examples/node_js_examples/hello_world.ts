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

// test_array()
// generics, a feature only in TS. Gives you more idea of what is being passed in
function getFirstElement<T>(arr: T[]): T {
    return arr[0];
}
let arr = [1,2,3];
console.log(getFirstElement(arr));

// Enum class
enum DriverCarStatus {
  POWER_COMM_OK = 0,
  GROUP_OPERATION = 1,
  UP_DIRECTION = 2,
  DOWN_DIRECTION = 3,
  DOOR_FULLY_OPEN = 4,
  DOOR_FULLY_CLOSED = 5,
  FIRE_SERVICE = 6,
  CODE_BLUE = 7,
  INSPECTION_OPERATIOM = 8,
  REAR_DOOR_FULLY_OPEN = 9, // TODO: Test this
  REAR_DOOR_FULLY_CLOSED = 10, // TODO: Test this
}
let arr3 = Array.from({length: 16}, (_, i) => i);
for (let i = 0; i < arr3.length; i++) {
    const status: string | undefined = DriverCarStatus[i];
    console.log(status, i)
}
