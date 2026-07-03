import os
import glob
import re
import xml.etree.ElementTree as ET
import base64
import numpy as np
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def decode_vtk_base64(element, dtype):
    text = element.text.strip()
    idx = text.find('=') + 1
    header_b64 = text[:idx]
    data_b64 = text[idx:]
    
    header_b64 += '=' * (-len(header_b64) % 4)
    data_b64 += '=' * (-len(data_b64) % 4)
    
    header_bytes = base64.b64decode(header_b64)
    size = struct.unpack('<Q', header_bytes[:8])[0]
    
    data_bytes = base64.b64decode(data_b64)
    return np.frombuffer(data_bytes[:size], dtype=dtype)

def parse_vtu(vtu_path):
    tree = ET.parse(vtu_path)
    root = tree.getroot()
    
    piece = root.find('.//Piece')
    num_points = int(piece.attrib['NumberOfPoints'])
    num_cells = int(piece.attrib['NumberOfCells'])
    
    # Extract points
    points_elem = piece.find('.//Points/DataArray')
    points = decode_vtk_base64(points_elem, np.float64).reshape(-1, 3)
    
    # Extract cells
    cells_elem = piece.find('.//Cells')
    conn = decode_vtk_base64(cells_elem.find(".//DataArray[@Name='connectivity']"), np.int64)
    offsets = decode_vtk_base64(cells_elem.find(".//DataArray[@Name='offsets']"), np.int64)
    
    # Extract cell data
    celldata_elem = piece.find('.//CellData')
    entity_type = decode_vtk_base64(celldata_elem.find(".//DataArray[@Name='EntityType']"), np.float64)
    
    # Parse cells into lists of point indices
    cells = []
    start = 0
    for offset in offsets:
        cells.append(conn[start:offset])
        start = offset
        
    return points, cells, entity_type

def plot_forces():
    print("Plotting aerodynamic forces over time (PDF)...")
    csv_path = 'output/forces.csv'
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        return
        
    data = np.genfromtxt(csv_path, delimiter=',', skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
        
    steps = data[:, 0]
    time = data[:, 1]
    fx = data[:, 2]
    fy = data[:, 3]
    fz = data[:, 4]
    
    # Clean, professional scientific plot styles
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.size'] = 11
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6.5), sharex=True)
    
    # Drag/In-plane forces
    ax1.plot(time, fx, label='$F_x$ (Drag / Axial)', color='#d35400', linewidth=1.2)
    ax1.plot(time, fy, label='$F_y$ (Side Force)', color='#2980b9', linewidth=1.2)
    ax1.set_ylabel('In-Plane Force [N]', color='#2c3e50', fontsize=11)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', frameon=True, fontsize=10)
    ax1.tick_params(colors='#2c3e50', labelsize=10)
    
    # Lift/Thrust force
    ax2.plot(time, fz, label='$F_z$ (Thrust / Lift)', color='#27ae60', linewidth=1.5)
    ax2.set_xlabel('Time [s]', color='#2c3e50', fontsize=11)
    ax2.set_ylabel('Thrust Force [N]', color='#2c3e50', fontsize=11)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', frameon=True, fontsize=10)
    ax2.tick_params(colors='#2c3e50', labelsize=10)
    
    plt.tight_layout()
    os.makedirs('assets', exist_ok=True)
    plt.savefig('assets/forces_vs_time.pdf', bbox_inches='tight')
    plt.savefig('assets/forces_vs_time.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Saved force history plot to assets/forces_vs_time.pdf and assets/forces_vs_time.png")

def create_animation():
    print("Searching for solution VTU files...")
    files = glob.glob('output/solution_*.vtu')
    if not files:
        print("No solution files found in output/.")
        return
        
    # Sort files by step number
    def get_step(filepath):
        match = re.search(r'solution_(\d+)\.vtu', filepath)
        return int(match.group(1)) if match else -1
        
    files.sort(key=get_step)
    
    # Downsample frames if there are too many (to speed up GIF creation)
    max_frames = 30
    if len(files) > max_frames:
        indices = np.linspace(0, len(files) - 1, max_frames, dtype=int)
        files = [files[i] for i in indices]
        
    print("Calculating global axes limits across all files...")
    all_mins = []
    all_maxs = []
    for file_path in files:
        points, _, _ = parse_vtu(file_path)
        all_mins.append(points.min(axis=0))
        all_maxs.append(points.max(axis=0))
        
    global_min = np.array(all_mins).min(axis=0)
    global_max = np.array(all_maxs).max(axis=0)
    
    # Calculate equal aspect range statically
    max_range = (global_max - global_min).max() / 2.0
    mid_x = (global_max[0] + global_min[0]) * 0.5
    mid_y = (global_max[1] + global_min[1]) * 0.5
    mid_z = (global_max[2] + global_min[2]) * 0.5
    
    print(f"Generating animation using {len(files)} frames...")
    
    fig = plt.figure(figsize=(10, 7), dpi=120)
    ax = fig.add_subplot(111, projection='3d')
    
    def update(frame_idx):
        ax.clear()
        ax.set_facecolor('#ffffff')
        fig.patch.set_facecolor('#ffffff')
        
        file_path = files[frame_idx]
        step_num = get_step(file_path)
        
        points, cells, entity_type = parse_vtu(file_path)
        
        vorton_xs, vorton_ys, vorton_zs = [], [], []
        
        for idx, cell in enumerate(cells):
            ent = entity_type[idx]
            if ent == 0:  # Blade
                if len(cell) == 4:
                    cell_pts = np.append(cell, cell[0])
                    pts = points[cell_pts]
                    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color='#34495e', linewidth=1.2, zorder=10)
            elif ent == 1:  # Wake Panel
                if len(cell) == 4:
                    cell_pts = np.append(cell, cell[0])
                    pts = points[cell_pts]
                    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color='#27ae60', linewidth=0.5, alpha=0.4, zorder=5)
            elif ent == 2:  # Vorton
                vorton_xs.append(points[cell[0]][0])
                vorton_ys.append(points[cell[0]][1])
                vorton_zs.append(points[cell[0]][2])
                
        if vorton_xs:
            ax.scatter(vorton_xs, vorton_ys, vorton_zs, color='#e74c3c', s=3, alpha=0.5, zorder=1)
            
        ax.set_title(f'UVLM Rotor & Wake Simulation - Step {step_num}', color='#2c3e50', fontsize=12)
        ax.set_xlabel('X [m]', color='#2c3e50')
        ax.set_ylabel('Y [m]', color='#2c3e50')
        ax.set_zlabel('Z [m]', color='#2c3e50')
        
        ax.xaxis.label.set_color('#2c3e50')
        ax.yaxis.label.set_color('#2c3e50')
        ax.zaxis.label.set_color('#2c3e50')
        ax.tick_params(colors='#2c3e50')
        ax.grid(True, color='#bdc3c7', linestyle='--')
        
        # Apply static global limits
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        ax.view_init(elev=20, azim=-55)
        print(f"  Rendered frame {frame_idx + 1}/{len(files)} (Step {step_num})")
        
    ani = animation.FuncAnimation(fig, update, frames=len(files), interval=150)
    
    output_gif = 'assets/rotor_simulation.gif'
    try:
        # Save as GIF using Pillow writer (always available with matplotlib)
        ani.save(output_gif, writer='pillow', fps=6)
        print(f"Saved simulation animation to {output_gif}!")
    except Exception as e:
        print(f"Error saving animation: {e}")
    plt.close()

if __name__ == '__main__':
    plot_forces()
    create_animation()
