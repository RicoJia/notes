/**
 * Imports
 * - CommonJS has require()
 * - ECMA import has import {} from ...
 *  - Default import, export: when you have only one thing to export, you can do:
 *  - export default function(){}
 *    - then in import, import someRandomName from module
 *  - Named imports with {}:
 *      - import {someRandomName} from module
 * */
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