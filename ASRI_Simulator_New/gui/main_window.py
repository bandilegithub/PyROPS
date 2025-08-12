"""
GUI components for the ASRI Simulator application.
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import webbrowser
import os
from typing import Any
from config.settings_manager import AppSettingsManager
from PIL import Image, ImageTk


class ASRIMainWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.settings_manager = AppSettingsManager()
        self.setup_main_window()
        self.create_main_interface()

    def setup_main_window(self):
        """Setup the main window properties"""
        self.root.title("PyROPS v2.4.0 - ASRI Propulsion and Automation Simulation Software")
        self.root.geometry('650x700+540+200')
        self.root.configure(bg='#f5f5f5')
        self.root.resizable(True, True)
        self.root.minsize(600, 650)

        # Create menu
        self.create_menu()

    def create_menu(self):
        """Create the menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Load Project", command=self.load_file)
        file_menu.add_command(label="Save Project", command=self.save_file)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.exit_app)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="Website", command=self.open_website)
        help_menu.add_command(label="Support", command=self.show_support)
        help_menu.add_command(label="About", command=self.show_about)

    def create_main_interface(self):
        """Create the main interface with logo and buttons"""
        # Main frame
        main_frame = tk.Frame(self.root, bg='#f5f5f5')
        main_frame.pack(fill='both', expand=True)

        # Try to load and display the ASRI image
        try:
            image_path = os.path.join(os.path.dirname(__file__), '..', 'ASRI.jpg')
            if os.path.exists(image_path):
                image = Image.open(image_path)
                # Resize image to fit the window nicely
                image = image.resize((486, 200), Image.Resampling.LANCZOS)
                self.photo = ImageTk.PhotoImage(image)

                # Create image label
                image_label = tk.Label(main_frame, image=self.photo, bg='#f5f5f5')
                image_label.pack(pady=(0, 10))
            else:
                # Fallback to text title if image not found
                self.create_text_header(main_frame)
        except Exception as e:
            print(f"Could not load image: {e}")
            # Fallback to text title
            self.create_text_header(main_frame)

        # Create beautiful organized button sections
        self.create_button_sections(main_frame)

    def create_text_header(self, parent):
        """Create a text header as fallback when image is not available"""
        header_frame = tk.Frame(parent, bg='#f5f5f5')
        header_frame.pack(pady=(20, 10))

        title_label = tk.Label(header_frame,
                              text="ASRI SIMULATOR",
                              font=('Arial', 24, 'bold'),
                              fg='#2c3e50',
                              bg='#f5f5f5')
        title_label.pack()

        subtitle_label = tk.Label(header_frame,
                                 text="PyROPS v2.4.0 - Propulsion and Automation Simulation",
                                 font=('Arial', 12),
                                 fg='#7f8c8d',
                                 bg='#f5f5f5')
        subtitle_label.pack()
    def create_button_sections(self, parent):
        """Create organized button sections"""
        # Main container for buttons
        button_container = tk.Frame(parent, bg='#f5f5f5')
        button_container.pack(fill='both', expand=True, padx=20, pady=10)

        # Create two columns
        left_column = tk.Frame(button_container, bg='#f5f5f5')
        left_column.pack(side='left', fill='both', expand=True, padx=(0, 10))

        right_column = tk.Frame(button_container, bg='#f5f5f5')
        right_column.pack(side='right', fill='both', expand=True, padx=(10, 0))

        # Button style
        section_style = {
            'font': ('Arial', 10, 'bold'),
            'relief': 'raised',
            'bd': 1,
            'cursor': 'hand2',
            'width': 20,
            'height': 2
        }

        # Left Column - Simulation & Analysis
        self.create_section(left_column, "🚀 Simulation & Analysis", [
            ("Start Simulation", self.start_simulation, '#2c5aa0'),
            ("Run Monte Carlo", self.run_monte_carlo, '#1e3a5f'),
            ("View Results", self.view_results, '#4a90e2'),
            ("Graphics Viewer", self.open_graphics, '#6bb6ff')
        ], section_style)

        self.create_section(left_column, "📊 Data Analysis", [
            ("Dispersion Map", self.dispersion_map, '#2ecc71'),
            ("Dispersion Plots", self.dispersion_plots, '#27ae60'),
            ("Trajectory Analysis", self.trajectory_analysis, '#16a085'),
        ], section_style)

        # Right Column - Tools & Utilities
        self.create_section(right_column, "🔧 Design Tools", [
            ("Mass Properties Calc", self.mass_properties, '#e67e22'),
            ("Thrust Curve Fit", self.thrust_curve_fit, '#d35400'),
            ("Missile DATCOM", self.missile_datcom, '#f39c12'),
            ("Create Wind File", self.create_wind_file, '#f1c40f')
        ], section_style)

        self.create_section(right_column, "⚙️ System", [
            ("Settings", self.open_settings, '#95a5a6'),
            ("Documentation", self.show_support, '#7f8c8d'),
            ("Website", self.open_website, '#34495e'),
            ("Exit", self.exit_app, '#e74c3c')
        ], section_style)

    def create_section(self, parent, title, buttons, button_style):
        """Create a section with title and buttons"""
        # Section frame
        section_frame = tk.LabelFrame(parent, text=title,
                                     font=('Arial', 11, 'bold'),
                                     fg='#2c3e50',
                                     bg='#f5f5f5',
                                     relief='groove',
                                     bd=2,
                                     padx=10,
                                     pady=8)
        section_frame.pack(fill='x', pady=8)

        # Create buttons in the section
        for button_text, command, color in buttons:
            btn = tk.Button(section_frame,
                           text=button_text,
                           command=command,
                           bg=color,
                           fg='white',
                           activebackground=self.darken_color(color),
                           activeforeground='white',
                           **button_style)
            btn.pack(fill='x', pady=2)

    def darken_color(self, color):
        """Darken a color for hover effect"""
        color_map = {
            '#2c5aa0': '#1e3f73',
            '#1e3a5f': '#152d47',
            '#4a90e2': '#357abd',
            '#6bb6ff': '#4a9bff',
            '#2ecc71': '#25a25a',
            '#27ae60': '#1e8b4e',
            '#16a085': '#128a76',
            '#e67e22': '#cc6a1c',
            '#d35400': '#b8470e',
            '#f39c12': '#d4860e',
            '#f1c40f': '#d4ac0d',
            '#95a5a6': '#7f8c8d',
            '#7f8c8d': '#6c7b7d',
            '#34495e': '#2c3e50',
            '#e74c3c': '#c0392b'
        }
        return color_map.get(color, color)

    # Enhanced button functions
    def start_simulation(self):
        """Open the simulation parameter window"""
        SimulationWindow(self.root, self.settings_manager)

    def run_monte_carlo(self):
        """Run Monte Carlo analysis"""
        messagebox.showinfo("Monte Carlo", "Starting Monte Carlo analysis...")

    def view_results(self):
        """View simulation results"""
        messagebox.showinfo("Results", "Opening results viewer...")

    def open_graphics(self):
        """Open graphics viewer"""
        messagebox.showinfo("Graphics", "Opening 3D graphics viewer...")

    def dispersion_map(self):
        """Show dispersion map"""
        messagebox.showinfo("Dispersion Map", "Opening dispersion map...")

    def dispersion_plots(self):
        """Show dispersion plots"""
        messagebox.showinfo("Dispersion Plots", "Opening dispersion plots...")

    def trajectory_analysis(self):
        """Open trajectory analysis"""
        messagebox.showinfo("Trajectory", "Opening trajectory analysis...")

    def mass_properties(self):
        """Open mass properties calculator"""
        messagebox.showinfo("Mass Properties", "Opening mass properties calculator...")

    def thrust_curve_fit(self):
        """Open thrust curve fitting tool"""
        messagebox.showinfo("Thrust Curve", "Opening thrust curve fitting tool...")

    def missile_datcom(self):
        """Open Missile DATCOM interface"""
        messagebox.showinfo("Missile DATCOM", "Opening Missile DATCOM interface...")

    def create_wind_file(self):
        """Create wind file"""
        messagebox.showinfo("Wind File", "Opening wind file creator...")

    def open_settings(self):
        """Open settings window"""
        SimulationWindow(self.root, self.settings_manager)

    @staticmethod
    def load_file():
        """Load a project file"""
        filepath = filedialog.askopenfilename(
            title="Load Project File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filepath:
            try:
                # Implementation for loading project
                messagebox.showinfo("Load", f"Loaded project from {filepath}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load project: {str(e)}")

    def save_file(self):
        """Save current project"""
        filepath = filedialog.asksaveasfilename(
            title="Save Project File",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filepath:
            try:
                # Implementation for saving project
                messagebox.showinfo("Save", f"Saved project to {filepath}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save project: {str(e)}")

    def open_website(self):
        """Open the ASRI website"""
        webbrowser.open('https://aerospace.ukzn.ac.za/')

    def show_support(self):
        """Show support information"""
        messagebox.showinfo("Support",
                           "For support and documentation:\n\n"
                           "• Visit our website\n"
                           "• Check the GitHub repository\n"
                           "• Contact: aerospace@ukzn.ac.za")

    def show_about(self):
        """Show about dialog"""
        messagebox.showinfo("About ASRI Simulator",
                           "ASRI Propulsion and Automation Simulation Software\n"
                           "Version 2.4.0\n\n"
                           "Developed by ASRI\n"
                           "University of KwaZulu-Natal\n\n"
                           "Advanced rocket simulation and analysis tool")

    def exit_app(self):
        """Exit the application"""
        if messagebox.askokcancel("Quit", "Are you sure you want to quit?"):
            self.root.quit()

    def run(self):
        """Start the main event loop"""
        self.root.mainloop()


class SimulationWindow:
    def __init__(self, parent, settings_manager):
        self.parent = parent
        self.settings_manager = settings_manager
        self.window = tk.Toplevel(parent)
        self.entries = {}
        self.setup_window()
        self.create_interface()

    def setup_window(self):
        """Setup the simulation window"""
        self.window.title("ASRI Simulation Parameters")
        self.window.geometry('1600x900')
        self.window.state('zoomed')
        self.window.configure(bg='#f8f9fa')

    def create_interface(self):
        """Create the parameter input interface"""
        # Main container
        main_container = tk.Frame(self.window, bg='#f8f9fa')
        main_container.pack(fill='both', expand=True, padx=10, pady=10)

        # Create notebook for organized tabs
        notebook = ttk.Notebook(main_container)
        notebook.pack(fill='both', expand=True)

        # Create tabs
        self.create_simulation_tab(notebook)
        self.create_rocket_tab(notebook)
        self.create_monte_carlo_tab(notebook)
        self.create_advanced_tab(notebook)

        # Control buttons frame
        control_frame = tk.Frame(main_container, bg='#f8f9fa')
        control_frame.pack(fill='x', pady=(10, 0))

        self.create_control_buttons(control_frame)

    def create_simulation_tab(self, notebook):
        """Create simulation parameters tab - core parameters needed to run simulation"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Simulation")

        # Create scrollable frame
        canvas = tk.Canvas(frame)
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.sim_vars = {}

        # Core Simulation Parameters - Essential for running any simulation
        sim_frame = ttk.LabelFrame(scrollable_frame, text="Core Simulation Parameters", padding="10")
        sim_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        sim_params = [
            ("Maximum Simulation Time", "max_simulation_time", "s", 1200.0),
            ("Time Step Size", "time_step_size", "s", 0.02),
            ("Number of Runs", "number_runs", "-", 1),
            ("Body State", "body_state", "-", 1),
        ]

        for i, (label, var_name, unit, default) in enumerate(sim_params):
            ttk.Label(sim_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=2)
            self.sim_vars[var_name] = tk.StringVar(value=str(default))
            entry = ttk.Entry(sim_frame, textvariable=self.sim_vars[var_name], width=15, justify='right')
            entry.grid(row=i, column=1, padx=5, pady=2)
            ttk.Label(sim_frame, text=unit).grid(row=i, column=2, sticky=tk.W, pady=2)

        # Launch Conditions - Where and how the rocket is launched
        launch_frame = ttk.LabelFrame(scrollable_frame, text="Launch Conditions", padding="10")
        launch_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        launch_params = [
            ("Launch Latitude", "launch_latitude", "deg", -34.600),
            ("Launch Longitude", "launch_longitude", "deg", 20.300),
            ("Launch Altitude", "launch_altitude", "m", 0.000),
            ("Launch Elevation", "launch_elevation", "deg", 80.000),
            ("Launch Azimuth", "launch_azimuth", "deg", -100.000),
            ("Launch Rail Length", "launch_rail_length", "m", 7.0),
        ]

        for i, (label, var_name, unit, default) in enumerate(launch_params):
            ttk.Label(launch_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=2)
            self.sim_vars[var_name] = tk.StringVar(value=str(default))
            entry = ttk.Entry(launch_frame, textvariable=self.sim_vars[var_name], width=15, justify='right')
            entry.grid(row=i, column=1, padx=5, pady=2)
            ttk.Label(launch_frame, text=unit).grid(row=i, column=2, sticky=tk.W, pady=2)

    def create_rocket_tab(self, notebook):
        """Create the rocket parameters tab"""
        tab = ttk.Frame(notebook)
        notebook.add(tab, text="Rocket Parameters")

        # Create scrollable frame
        canvas = tk.Canvas(tab, bg='#f8f9fa')
        scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Rocket geometry parameters
        self.create_parameter_section(scrollable_frame, "Rocket Geometry", [
            ("Rocket Body Radius", "body_radius", 0.087, "m"),
            ("Rocket Body Length", "body_length", 4.920, "m"),
            ("Launch Rail Length", "rail_length", 7.0, "m"),
            ("Nozzle Exit Area", "nozzle_area", 0.007056, "m²"),
        ])

        # Mass properties
        self.create_parameter_section(scrollable_frame, "Mass Properties", [
            ("CAD Mass (Dry)", "cad_mass", 49.35224149, "kg"),
            ("CAD COM X", "cad_com_x", 1.386632, "m"),
            ("CAD COM Y", "cad_com_y", 0.0, "m"),
            ("CAD COM Z", "cad_com_z", 0.0, "m"),
            ("CAD MOI X", "cad_moi_x", 0.04023116, "kg⋅m²"),
            ("CAD MOI Y", "cad_moi_y", 180.8297, "kg⋅m²"),
            ("CAD MOI Z", "cad_moi_z", 180.8297, "kg⋅m²"),
        ])

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_monte_carlo_tab(self, notebook):
        """Create the Monte Carlo parameters tab"""
        tab = ttk.Frame(notebook)
        notebook.add(tab, text="Monte Carlo Analysis")

        # Create scrollable frame
        canvas = tk.Canvas(tab, bg='#f8f9fa')
        scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Monte Carlo settings
        self.create_parameter_section(scrollable_frame, "Monte Carlo Settings", [
            ("Number of Runs", "mc_runs", 1000, "runs"),
            ("Monte Carlo Enable", "mc_enable", 1, "on=1 off=0"),
            ("Detailed Output", "mc_detailed", 1, "on=1 off=0"),
        ])

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_advanced_tab(self, notebook):
        """Create the advanced parameters tab"""
        tab = ttk.Frame(notebook)
        notebook.add(tab, text="Advanced Settings")

        # Create scrollable frame
        canvas = tk.Canvas(tab, bg='#f8f9fa')
        scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Solver settings
        self.create_parameter_section(scrollable_frame, "Solver Settings", [
            ("8th Order Solver", "solver_8th", 1, "on=1 off=0"),
            ("Relative Tolerance", "rel_tolerance", 1.0, "-"),
            ("Absolute Tolerance", "abs_tolerance", 1.0, "-"),
            ("First Step Size", "first_step", 0.02, "s"),
            ("Max Step Size", "max_step", 0.02, "s"),
        ])

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_parameter_section(self, parent, title, parameters):
        """Create a section of parameters with a title"""
        # Section frame
        section_frame = ttk.LabelFrame(parent, text=title, padding="10")
        section_frame.pack(fill='x', padx=10, pady=5)

        for i, (label, key, default, unit) in enumerate(parameters):
            row_frame = tk.Frame(section_frame, bg='#f8f9fa')
            row_frame.pack(fill='x', pady=2)

            # Label
            label_widget = tk.Label(row_frame, text=label,
                                  anchor='w', width=30,
                                  bg='#f8f9fa', font=('Arial', 10))
            label_widget.pack(side='left')

            # Entry
            entry = tk.Entry(row_frame, width=20, font=('Arial', 10),
                           justify='right', relief='solid', bd=1)
            entry.insert(0, str(default))
            entry.pack(side='left', padx=(5, 5))

            # Unit label
            unit_label = tk.Label(row_frame, text=unit,
                                anchor='w', width=8,
                                bg='#f8f9fa', font=('Arial', 10))
            unit_label.pack(side='left')

            # Store entry reference
            self.entries[key] = entry

    def create_control_buttons(self, parent):
        """Create control buttons"""
        button_style = {
            'font': ('Arial', 10, 'bold'),
            'width': 18,
            'height': 2,
            'relief': 'raised',
            'bd': 2,
            'cursor': 'hand2'
        }

        # Left side buttons
        left_frame = tk.Frame(parent, bg='#f8f9fa')
        left_frame.pack(side='left')

        load_btn = tk.Button(left_frame, text="Load Settings",
                           bg='#17a2b8', fg='white',
                           command=self.load_settings, **button_style)
        load_btn.pack(side='left', padx=5)

        save_btn = tk.Button(left_frame, text="Save Settings",
                           bg='#28a745', fg='white',
                           command=self.save_settings, **button_style)
        save_btn.pack(side='left', padx=5)

        refresh_btn = tk.Button(left_frame, text="Refresh",
                              bg='#6c757d', fg='white',
                              command=self.refresh_values, **button_style)
        refresh_btn.pack(side='left', padx=5)

        # Right side buttons
        right_frame = tk.Frame(parent, bg='#f8f9fa')
        right_frame.pack(side='right')

        simulate_btn = tk.Button(right_frame, text="Run Simulation",
                               bg='#007bff', fg='white',
                               command=self.run_simulation, **button_style)
        simulate_btn.pack(side='left', padx=5)

        plot_btn = tk.Button(right_frame, text="View Results",
                           bg='#fd7e14', fg='white',
                           command=self.view_results, **button_style)
        plot_btn.pack(side='left', padx=5)

        close_btn = tk.Button(right_frame, text="Close",
                            bg='#dc3545', fg='white',
                            command=self.close_window, **button_style)
        close_btn.pack(side='left', padx=5)

    def load_settings(self):
        """Load settings from file"""
        try:
            settings = self.settings_manager.get_all_settings()
            for key, entry in self.entries.items():
                if key in settings:
                    entry.delete(0, tk.END)
                    entry.insert(0, str(settings[key]))
            messagebox.showinfo("Success", "Settings loaded successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load settings: {str(e)}")

    def save_settings(self):
        """Save current settings to file"""
        try:
            settings = {}
            for key, entry in self.entries.items():
                try:
                    value = float(entry.get())
                    settings[key] = value
                except ValueError:
                    settings[key] = entry.get()

            # Update settings
            for key, value in settings.items():
                self.settings_manager.update_setting(key, value)

            messagebox.showinfo("Success", "Settings saved successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save settings: {str(e)}")

    def refresh_values(self):
        """Refresh all values to defaults"""
        if messagebox.askyesno("Refresh", "Reset all values to defaults?"):
            # Reset to default values - this would reload from config
            self.load_settings()

    def run_simulation(self):
        """Run the simulation"""
        try:
            # Get all values
            params = {}
            for key, entry in self.entries.items():
                try:
                    params[key] = float(entry.get())
                except ValueError:
                    params[key] = entry.get()

            # Here you would call the actual simulation engine
            messagebox.showinfo("Simulation",
                              "Simulation started with current parameters.\n"
                              "Check console for progress updates.")

        except Exception as e:
            messagebox.showerror("Error", f"Simulation failed: {str(e)}")

    def view_results(self):
        """View simulation results"""
        messagebox.showinfo("Results", "Opening results viewer...")

    def close_window(self):
        """Close the simulation window"""
        self.window.destroy()


# Main application function
def main():
    app = ASRIMainWindow()
    app.run()


if __name__ == "__main__":
    main()
