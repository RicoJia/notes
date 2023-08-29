// import logo from './logo.svg';
// import './App.css';

// function App() {
//   return (
//     <div className="App">
//       <header className="App-header">
//         <img src={logo} className="App-logo" alt="logo" />
//         <p>
//           Edit <code>src/App.js</code> and save to reload.
//         </p>
//         <a
//           className="App-link"
//           href="https://reactjs.org"
//           target="_blank"
//           rel="noopener noreferrer"
//         >
//         Rico says: Slas
//         </a>
//       </header>
//     </div>
//   );
// }

// export default App;
import React, { useState, useEffect } from 'react';
import './App.css';

function App() {
  /*
    - useState: react hook to initialize variable isVisible to true
    and function setIsVisible to update isVisible. Note setIsVisible has not been defined yet
  */
  const [isVisible, setIsVisible] = useState(true);

  /*
  - useEffect: react hook to run side effects, like fetching data, etc.
  effectively, mounting the component, update it, and unmount it.
  */
  useEffect(() => {
    /*
    setInterval returns an ID of the interval.
    */
    const interval = setInterval(() => {
      /*
      - setIsVisible is defined to be prev => !prev. The passed in value is the registered 
      value isVisible. The result will go back to isVisible, too. 
      This is a state setter
      */
      setIsVisible(prev => !prev);
    }, 2000); // 500ms for 2 toggles = 1Hz

    /*
    - this effectively registers setInterval in the JS event loop. clearInterval is built-in JS
    function, prevent leak
    */
    return () => clearInterval(interval); // Cleanup interval on component unmount
  }, []);

  return (
    <div className="App">
      <div className="rectangle-container">
        {Array(5).fill(null).map((_, idx) => (
          <div
            key={idx}
            className={`rectangle ${isVisible ? 'visible' : 'hidden'}`}
          ></div>
        ))}
      </div>
    </div>
  );
}

export default App;