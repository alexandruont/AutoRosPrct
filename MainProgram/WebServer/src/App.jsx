import { Canvas } from '@react-three/fiber'
import CubeScene from './components/CubeScene'
import Header from './Header'
import './App.css'

function App() {
    return (
        <>
        <Header />
        <Canvas className='MainCanva' camera={{ position: [3, 3, 3] }}>
            <CubeScene />
        </Canvas>
        </>
    )
}

export default App
