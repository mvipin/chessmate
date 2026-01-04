# SCARA Robot Mesh Export Guide

## Fusion 360 STL Export Instructions

### **Step 1: Prepare Your F360 Model**
1. Ensure each component is properly named:
   - Base → `base_link`
   - First arm → `link1` 
   - Second arm → `link2`
   - End effector → `end_effector`

### **Step 2: Export Visual Meshes (High Detail)**
For each component:
1. Right-click the component in the browser
2. Select "Save As Mesh"
3. Choose STL format
4. Set refinement to **Medium** or **High**
5. Save to `visual/` folder with naming:
   - `base_link_visual.stl`
   - `link1_visual.stl`
   - `link2_visual.stl`
   - `end_effector_visual.stl`

### **Step 3: Export Collision Meshes (Low Detail)**
For each component:
1. Right-click the component in the browser
2. Select "Save As Mesh"
3. Choose STL format
4. Set refinement to **Low** or **Medium**
5. Save to `collision/` folder with naming:
   - `base_link_collision.stl`
   - `link1_collision.stl`
   - `link2_collision.stl`
   - `end_effector_collision.stl`

### **Step 4: Enable Meshes in URDF**
Once STL files are exported, edit the URDF:
```xml
<xacro:property name="use_meshes" value="true" />
```

## File Structure
```
meshes/
├── visual/           # High-detail meshes for visualization
│   ├── base_link_visual.stl
│   ├── link1_visual.stl
│   ├── link2_visual.stl
│   └── end_effector_visual.stl
├── collision/        # Low-detail meshes for collision detection
│   ├── base_link_collision.stl
│   ├── link1_collision.stl
│   ├── link2_collision.stl
│   └── end_effector_collision.stl
└── README.md
```

## Coordinate System Notes
- F360 exports in mm, URDF uses meters (scale factor: 0.001)
- Ensure F360 coordinate system matches URDF expectations
- X-axis should point forward along the link
- Z-axis should point up for the base

## Quality Settings
- **Visual meshes**: Medium/High detail for realistic appearance
- **Collision meshes**: Low detail for performance
- **File size**: Keep visual meshes under 1MB each if possible

## Testing
After exporting:
1. Set `use_meshes="true"` in the URDF
2. Rebuild: `colcon build --packages-select chessmate_description`
3. Launch: `ros2 launch chessmate_description display.launch.py`
4. Verify meshes load correctly in RViz2

## Troubleshooting
- **Mesh not visible**: Check file paths and scaling
- **Wrong orientation**: Verify F360 coordinate system
- **Performance issues**: Use lower detail collision meshes
- **File not found**: Ensure meshes are installed with `colcon build`
