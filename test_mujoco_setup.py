#!/usr/bin/env python3
"""
Test MuJoCo Setup

This script tests different GL backends and helps diagnose rendering issues.

Usage:
    python test_mujoco_setup.py
"""

import os
import sys

def test_backend(backend_name):
    """Test a specific GL backend."""
    print(f"\n{'='*50}")
    print(f"Testing backend: {backend_name}")
    print('='*50)
    
    # Must set before importing mujoco
    os.environ['MUJOCO_GL'] = backend_name
    
    # Force reimport
    if 'mujoco' in sys.modules:
        del sys.modules['mujoco']
    
    try:
        import mujoco
        print(f"✓ MuJoCo imported successfully (version {mujoco.__version__})")
        
        # Try to create a simple model
        xml = """
        <mujoco>
            <worldbody>
                <body>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
        </mujoco>
        """
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_step(model, data)
        print("✓ Simulation works")
        
        # Try rendering
        try:
            renderer = mujoco.Renderer(model, height=240, width=320)
            renderer.update_scene(data)
            pixels = renderer.render()
            print(f"✓ Rendering works ({pixels.shape})")
            renderer.close()
            return True
        except Exception as e:
            print(f"✗ Rendering failed: {e}")
            return False
            
    except Exception as e:
        print(f"✗ Failed: {e}")
        return False


def main():
    print("MuJoCo Setup Tester")
    print("==================")
    print(f"Python: {sys.version}")
    print(f"DISPLAY: {os.environ.get('DISPLAY', 'not set')}")
    
    # Check for required libraries
    print("\nChecking system libraries...")
    
    import subprocess
    libs = {
        'libosmesa': 'OSMesa (software rendering)',
        'libEGL': 'EGL (GPU rendering)',
        'libGL': 'OpenGL',
    }
    
    for lib, desc in libs.items():
        try:
            result = subprocess.run(['ldconfig', '-p'], capture_output=True, text=True)
            if lib in result.stdout:
                print(f"  ✓ {desc} ({lib}) - found")
            else:
                print(f"  ? {desc} ({lib}) - not found in ldconfig")
        except:
            print(f"  ? {desc} ({lib}) - couldn't check")
    
    # Test backends
    backends = ['osmesa', 'egl', 'glfw']
    working = []
    
    for backend in backends:
        # Need to restart Python process to properly test each backend
        # For now, just try the current one
        pass
    
    # Test current/default backend
    print("\n" + "="*50)
    print("Testing with auto-detected backend...")
    print("="*50)
    
    # Clear any existing mujoco import
    for mod in list(sys.modules.keys()):
        if 'mujoco' in mod or 'OpenGL' in mod or 'glfw' in mod:
            del sys.modules[mod]
    
    # Try each backend
    for backend in ['osmesa', 'egl', 'glfw']:
        os.environ['MUJOCO_GL'] = backend
        
        # Clear modules
        for mod in list(sys.modules.keys()):
            if 'mujoco' in mod or 'OpenGL' in mod or 'glfw' in mod:
                del sys.modules[mod]
        
        try:
            import mujoco
            
            xml = '<mujoco><worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>'
            model = mujoco.MjModel.from_xml_string(xml)
            data = mujoco.MjData(model)
            
            renderer = mujoco.Renderer(model, height=100, width=100)
            renderer.update_scene(data)
            pixels = renderer.render()
            renderer.close()
            
            print(f"\n✓ Backend '{backend}' works!")
            working.append(backend)
            
        except Exception as e:
            print(f"\n✗ Backend '{backend}' failed: {type(e).__name__}")
        
        # Cleanup
        for mod in list(sys.modules.keys()):
            if 'mujoco' in mod:
                del sys.modules[mod]
    
    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    
    if working:
        print(f"\nWorking backends: {', '.join(working)}")
        print(f"\nRecommended: Add this to the TOP of your script:\n")
        print(f"    import os")
        print(f"    os.environ['MUJOCO_GL'] = '{working[0]}'")
        print(f"\nOr run with:")
        print(f"    MUJOCO_GL={working[0]} python your_script.py")
    else:
        print("\nNo working backends found!")
        print("\nTry installing:")
        print("  sudo apt-get install libosmesa6-dev  # For osmesa")
        print("  sudo apt-get install libegl1-mesa-dev  # For EGL")


if __name__ == '__main__':
    main()
