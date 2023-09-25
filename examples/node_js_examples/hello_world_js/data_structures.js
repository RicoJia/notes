/**
 * ================ Array ================
 * array is an subtype of object, so you can do
 */
const test_arr_basics = () => {
    var arr = ["celos", "juego", "destino"];
    typeof arr;
    // creates a new array
    let new_arr = arr.map((i) => i+"rico");
    console.log("test array basics:", new_arr);

    // splice and remove IN PLACE
    // replace 2 elements start at index 1 with "haha"
    arr.splice(1,2,"haha")
    // guess what this does ;)
    arr.splice(1,2)
    console.log("What does array look like?", arr)

    arr.push("haha");
    arr.pop()
    console.log("Arrary is a stack. push and pop are LIFO: ", arr);

}
// test_arr_basics();

const test_arr_advanced_operations = () => {
    var arr = ["celos", "juego", "destino", "destino"];
    console.log('first index for celos: ', arr.indexOf("celos"));
    console.log('last index for destino: ', arr.lastIndexOf("destino"));
    console.log('array includes destino: ', arr.includes("destino"));
    console.log('slice [0,1): ', arr.slice(0,1));
    arr.unshift("nuevo");
    console.log('unshift is inserting an element at the index', arr);
    arr.shift();
    console.log('shift is removing the beginning element: ', arr);
    console.log('array concatenation using +, You get a STRING: ', typeof(arr + ["new", "element"]));
    console.log('array concatenation using ...: ', [...arr, ...["new", "element"]]);
}
// test_arr_advanced_operations();

const test_filter = () => {
    let arr = [true, false, true, true];
    let new_arr = arr.filter(x => x);
    console.log("test filter, removed falsy elements with filter: ", new_arr);
}
// test_filter();

const test_unpacking = () => {
    let arr = [a,b,c] = [1,2,3];
    console.log('unpacking an array: ', a, b, c);
    let obj = {name: 'foo', age: 99};
    let {name} = obj;
    console.log('unpacking an object if you have the field: ', name);

    let  foo = (a,b,c) => {
        console.log("unpacking an args in an arrow function, using ...: ", a, b, c);
    }
    // ... is aka the rest/spreading operator
    foo(...arr);
}
// test_unpacking();

const test_range = () => {
    // In js there's no built in range
    let start = 20, end = 0, step = -1; 
    let range_20_0 = Array.from({length: Math.abs(end - start)}, (_, i) => start + i * step);
    console.log('range: ', range_20_0);
    console.log('array reverse: ', range_20_0.reverse());
}
// test_range();

const test_some_every = () => {
    const arr = [1, 2, 3, 3];
    console.log('if some of array is >2: ', arr.some(i => i>2));
    console.log('if every of array is >2: ', arr.every(i => i>2));
}
// test_some_every();
// ================ String ================
const test_string = () => {
    let str = "hello world";
    str.property_wont_stay = "propery_wont stay";
    console.log('property wont stay: ', str.property_wont_stay);
    console.log('string length: ', str.length);

    const originalString = "   Hello, world!   ";
    console.log("removed extraneous spacing using trim(): ", originalString.trim());
    console.log('slicing [0,2): ', str.slice(0, 2));
    console.log('padding w/ spacing: ', str.padStart(str.length + 5, "paddddd"));
}
// test_string();

/**
 * - In Javascript we can't replace the string index directly, like
    Also, we can't do name_i[1:-1]
    name_i[0]=name_i[0].toUpperCase();
    - Also, strings are encoded in 2 bytes (UTF-16). Not great idea, cuz you can get half char,
     but becoming more common with emoji
 */
const test_string_replace = () => {
    // name is a host object's name, so otherwise it will overwrite
    var name_i = "hello"
    name_i = `${name_i[0].toUpperCase()}${name_i.slice(1, name_i.length)}`;
    name_i = name_i.charAt(0).toUpperCase() + name_i.slice(1);
    console.log(name_i.slice(0,3));
}

// ================ JSON ================
const test_json = () => {
    let obj = {1:2, 3:4}
    let json_str = JSON.stringify(obj);
    console.log("json string: ", json_str);
    console.log('json object: ', JSON.parse(json_str));
}
// test_json();


// ================ Maps ================
/**
 * Regular objects store keys as strings. Maps can store things as non-keys
 * But you must use Map.has, Map.set, Map.get
 */
const test_maps = () => {
    let m = new Map();
    m[1] = "one";
    // you are setting 1 as '1' in the object, NOT in the map
    console.log('map: ', m, " if map has 1 as key", m.has(1));
    m.set(2, "two");
    console.log('map: ', m, " if map has 2 as key", m.has(2), "value is: ", m.get(2));
}
// test_maps();
