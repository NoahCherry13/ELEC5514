import os
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def get_independent_variable(filename):
    """Parses the filename to determine the independent variable column."""
    if "rate-sweep" in filename:
        return "FlowRate_kbps"
    elif "dist-sweep" in filename:
        return "Radius"
    else:
        # Default to the first column if sweep type is not in filename
        return None

def plot_data(data_path, output_dir):
    """
    Reads a CSV file, identifies the independent variable from the filename,
    and generates plots for key metrics with descriptive labels and titles.
    """
    filename = data_path.name
    iv_col = get_independent_variable(filename)

    # Mappings for descriptive labels and titles
    label_map = {
        "Radius": "Radius (m)",
        "FlowRate_kbps": "Flow Rate (kbps)",
        "PDR_pct": "Packet Delivery Ratio (%)",
        "Throughput_kbps": "Throughput (kbps)",
        "Delay_ms": "End-to-End Delay (ms)"
    }
    
    topology_map = {
        "circle": "Ring Topology",
        "grid": "Grid Mesh Topology"
    }

    # Determine topology from filename
    topology_str = "Unknown Topology"
    for key, value in topology_map.items():
        if key in filename:
            topology_str = value
            break

    try:
        df = pd.read_csv(data_path)
    except FileNotFoundError:
        print(f"Error: File not found at {data_path}")
        return

    if iv_col is None:
        iv_col = df.columns[0]

    if iv_col not in df.columns:
        print(f"Error: Independent variable '{iv_col}' not found in {filename}")
        return

    dependent_vars = ["PDR_pct", "Throughput_kbps", "Delay_ms"]
    
    for dv_col in dependent_vars:
        if dv_col not in df.columns:
            print(f"Warning: Dependent variable '{dv_col}' not found in {filename}, skipping plot.")
            continue

        plt.figure(figsize=(10, 6))
        plt.plot(df[iv_col], df[dv_col], marker='o', linestyle='-')
        
        # Use mappings for descriptive titles and labels
        dv_label = label_map.get(dv_col, dv_col)
        iv_label = label_map.get(iv_col, iv_col)
        
        title = f"{dv_label} vs. {iv_label} for {topology_str}"
        plt.title(title)
        plt.xlabel(iv_label)
        plt.ylabel(dv_label)
        plt.grid(True)
        
        # Create a clean filename for the plot
        plot_filename = f"{Path(filename).stem}_{dv_col}_vs_{iv_col}.png"
        output_path = output_dir / plot_filename
        
        plt.savefig(output_path)
        plt.close()
        print(f"Saved plot to {output_path}")

def main():
    """
    Main function to find CSV files and generate plots.
    """
    project_root = Path(__file__).parent
    data_dir = project_root / "data"
    output_dir = project_root / "plots"
    
    # Create the output directory if it doesn't exist
    output_dir.mkdir(exist_ok=True)
    
    if not data_dir.exists():
        print(f"Error: Data directory not found at {data_dir}")
        return

    print(f"Searching for CSV files in {data_dir}...")
    for item in os.scandir(data_dir):
        if item.is_file() and item.name.endswith('.csv'):
            print(f"Processing {item.name}...")
            plot_data(Path(item.path), output_dir)

if __name__ == "__main__":
    main()