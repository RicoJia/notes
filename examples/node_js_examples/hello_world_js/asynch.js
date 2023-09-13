/*
- Asynchrony
    - Synchronous is not multi-threaded: two threads can be blocked by I/O (like networking)
        - Async is two threads can continue while being blocked.
        - So, synchronous + multi-threaded is great for CPU-intensive and I/O intensive work
    - Callback only coding suffers from Inversion of Control? (IoC), lack of linear reasonability
    - Ways to achieve asynchrony:
        - Promise(function) for execution
        - Callbacks: very common, but sometimes you just need to return a promise
- Create a promise
    - Promise((resolve, reject) => {do stuff, resolve/reject(value)})
        0. This returns another promise immediately. But its execution is in the background
        1. promise has 3 states: pending, fulfilled, rejected.
        2. resolve will set state to fulfilled, reject to rejected.
        3. Promise can't change state from fulfilled, or rejected.
        4. You can call the promise multiple times, if their state is not pending, 
            result will be returned immediately
        5. Can be simplified as Promise((resolve) => {do stuff, resolve/reject(value)})
    - Promise.then(result => {do_stuff, can return another promise})
        6. Can be omit Promise in then(): Promise(resolve => {resolve('1')}).
            then(result => result+'1' )
            - instead of then(result => Promise(resolve => resolve(result+'1')))
            
*/
const test_promise_solve = () => {
    setTimeout(()=>{console.log("setTimeout");}, 500);
    // 1. Create a promise that immediately resolves to 15. then set a callback
    // fiftenn_promise.then actualy returns promise<number>
    fifteen_promise = Promise.resolve(15);
    fifteen_promise.then(value => console.log(value));
    console.log(typeof fifteen_promise);
}

const test_promise_chaining = () => {
    // 2. Sample usage
    function simulateFetchingWebpage(){
        // when creating a promise, you get resolve and reject, 
        return new Promise((resolve, reject) => {
            setTimeout(() => {
                const data = "hola";
                console.log(data);
                resolve(data);
            }, 2000);
        });
    }

    // this already starts the execution of the promise
    simulateFetchingWebpage();
    console.log("after simulateFetchingWebpage");
    // reuse promise<any> again, but with another callback. 
    // Good practice: catch errors thrown too.
    simulateFetchingWebpage().then(
        // data comes from the resolve previosly
        data => {
            console.log("Can reuse the promise", data);
        }
    ).catch(
        error => console.log("error could be thrown, by rejected", error)
    )
}
// test_promise_chaining();
    

// 3. Reject
function testReject(){
    return new Promise(
        (_, reject) => reject(new Error("Fail"))
    ).catch(
        reason => {
                // here you will see the full tracestack
                console.log("Caught failure ", reason);
            return "nothing";
        }
    );
}
// testReject();


/**
 * Async function returns a promise, implicitly
 * - inside it, you return a value, that's resolved (see above). 
 * - If you throw an error, that's rejection
    - then, javascript will continue to other codes (like "haha")
    - await will make sure async function is executed first
 */
const test_fetch_url = () => {
    async function fetch_url(url){
        try{
            // pause the function until to wait for a promise to be resolved
            // await returns a value resolved from promise
            let response = await fetch("http://0.0.0.0:8000/");
            let data = await response.text();
            console.log("fetched html, ", data);
        }catch(e){
            console.log("an error occurred: " + e.message);
        }
    }
    fetch_url();
    console.log("THis prints before fetch_url");
}


/**
 * Advanced:
 *  - Promise.all([new Promise(), new Promise()]): return results from all promises
 *  - Promise.race([new Promise(), new Promise()]): return the fastest-executed result
 */
const test_all_and_race_promises = () => {
    // To see the result, you have to use then(result)
    Promise.all([
        new Promise(resolve => {
            setTimeout(() => {resolve('sleep 1s');}, 1000);
        }),
        new Promise(resolve => {
            setTimeout(() => {resolve('sleep 0.5s');}, 500);
        })
    ]).then(result => console.log(result)); // see [ 'sleep 1s', 'sleep 0.5s' ]

    // see sleep 0.5s
    Promise.race([
        new Promise(resolve => {
            setTimeout(() => {resolve('sleep 1s');}, 1000);
        }),
        new Promise(resolve => {
            setTimeout(() => {
               resolve('sleep 0.5s'); 
            }, 500);
        })
    ]).then(result => {console.log(result)});
}
test_all_and_race_promises();