import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, filedialog
import socket, threading, time, re, math
import numpy as np
from datetime import datetime
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from scipy import ndimage
from scipy.signal import savgol_filter


class Scanner3D:
    def __init__(self, root):
        self.root = root
        self.root.title("Scanner 3D")
        self.root.geometry("900x700")

        # Estado
        self.arduino_ip = "192.168.4.1"
        self.arduino_port = 80
        self.connected = False
        self.scanning = False

        # Variáveis
        self.file_type = tk.StringVar()
        self.fast_scan = tk.BooleanVar(value=False)
        self.raw_points = []
        self.processed_points = []

        # UI setup
        self.setup_ui()

        # Iniciar verificação de conexão
        threading.Thread(target=self.check_connection_loop, daemon=True).start()

    def setup_ui(self):
        # Estilo
        s = ttk.Style()
        s.configure("TButton", padding=10, font=('Arial', 12))
        s.configure("TLabel", font=('Arial', 12))
        s.configure("Header.TLabel", font=('Arial', 16, 'bold'))

        # Frame principal
        main = ttk.Frame(self.root, padding="10")
        main.pack(fill=tk.BOTH, expand=True)

        # Título e status
        ttk.Label(main, text="Scanner 3D", style="Header.TLabel").pack(pady=10)

        status_frame = ttk.Frame(main)
        status_frame.pack(pady=5, fill=tk.X)
        ttk.Label(status_frame, text="Status:").pack(side=tk.LEFT, padx=5)
        self.connection_label = ttk.Label(status_frame, text="Desconectado", foreground="red")
        self.connection_label.pack(side=tk.LEFT)

        # Botões principais
        btn_frame = ttk.Frame(main)
        btn_frame.pack(pady=10)
        ttk.Button(btn_frame, text="Calibrar", command=self.calibrar).pack(side=tk.LEFT, padx=10)
        self.iniciar_btn = ttk.Button(btn_frame, text="Iniciar", command=self.iniciar_ou_cancelar)
        self.iniciar_btn.pack(side=tk.LEFT, padx=10)

        # Opções de scan
        self.file_options_frame = ttk.LabelFrame(main, text="Opções de Scanner")
        ttk.Button(self.file_options_frame, text="Guardar como .txt",
                   command=lambda: self.selecionar_tipo_arquivo("txt")).pack(side=tk.LEFT, padx=10, pady=10)
        ttk.Button(self.file_options_frame, text="Guardar como .STL",
                   command=lambda: self.selecionar_tipo_arquivo("stl")).pack(side=tk.LEFT, padx=10, pady=10)
        ttk.Checkbutton(self.file_options_frame, text="Fast Scan",
                        variable=self.fast_scan).pack(side=tk.LEFT, padx=10)

        # Status message
        self.message_label = ttk.Label(main, text="")
        self.message_label.pack(pady=5)

        # Botões de processamento
        proc_frame = ttk.Frame(main)
        proc_frame.pack(pady=5, fill=tk.X)

        # Criar botões com mesma configuração
        self.btn_config = [
            ("Visualizar na GUI", self.create_embedded_visualization),
            ("Processar Pontos", self.process_point_cloud),
            ("Gerar Sólido 3D", self.generate_solid_model)
        ]

        self.proc_buttons = []
        for text, cmd in self.btn_config:
            btn = ttk.Button(proc_frame, text=text, command=cmd, state=tk.DISABLED)
            btn.pack(side=tk.LEFT, padx=5)
            self.proc_buttons.append(btn)

        # Área de visualização
        self.viz_frame = ttk.LabelFrame(main, text="Visualização 3D")
        self.viz_frame.pack(pady=10, fill=tk.BOTH, expand=True)

        # Log
        log_frame = ttk.LabelFrame(main, text="Log e dados")
        log_frame.pack(pady=5, fill=tk.X)

        self.log_area = scrolledtext.ScrolledText(log_frame, height=10)
        self.log_area.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Botão para guardar dados
        self.save_btn = ttk.Button(main, text="Guardar Dados", command=self.guardar_dados, state=tk.DISABLED)
        self.save_btn.pack(pady=10)

    def check_connection_loop(self):
        """Verifica conexão com Arduino periodicamente"""
        while True:
            if not self.scanning:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.settimeout(2)
                        result = s.connect_ex((self.arduino_ip, self.arduino_port))

                    new_status = (result == 0)
                    if new_status != self.connected:
                        self.connected = new_status
                        self.root.after(0, lambda c=new_status: self.connection_label.config(
                            text="Conectado" if c else "Desconectado",
                            foreground="green" if c else "red"))
                except:
                    if self.connected:
                        self.connected = False
                        self.root.after(0, lambda: self.connection_label.config(
                            text="Desconectado", foreground="red"))
            time.sleep(5)

    def send_command(self, command, timeout=5):
        """Envia comando para o Arduino"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(timeout)
                s.connect((self.arduino_ip, self.arduino_port))
                s.sendall(command.encode('utf-8'))
                return s.recv(1024).decode('utf-8')
        except Exception as e:
            print(f"Erro ao enviar comando: {e}")
            raise e

    def iniciar_ou_cancelar(self):
        """Alterna entre iniciar scan e cancelar scan"""
        if not self.scanning:
            if not self.connected:
                messagebox.showerror("Erro", "Arduino não está conectado!")
                return
            self.file_options_frame.pack(pady=10, fill=tk.X)
        else:
            threading.Thread(target=lambda: self.send_command("cancelar", 3), daemon=True).start()
            self.log_area.insert(tk.END, "\nA cancelar...\n")
            self.log_area.yview(tk.END)

    def calibrar(self):
        """Inicia calibração numa thread separada"""
        if not self.connected:
            messagebox.showerror("Erro", "Arduino não está conectado!")
            return

        def run_calibration():
            try:
                self.root.after(0, lambda: self.message_label.config(
                    text="Calibração iniciada...", foreground="blue"))

                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(30)
                    s.connect((self.arduino_ip, self.arduino_port))
                    s.sendall("calibrar".encode('utf-8'))

                    buffer = ""
                    while True:
                        try:
                            data = s.recv(1024).decode('utf-8')
                            if not data:
                                break

                            buffer += data

                            if "CALIBRACAO_CONCLUIDA" in buffer:
                                self.root.after(0, lambda: self.message_label.config(
                                    text="Calibração concluída com sucesso!", foreground="green"))
                                break
                            elif "CALIBRACAO_INICIADA" in buffer:
                                self.root.after(0, lambda: self.message_label.config(
                                    text="Calibração em andamento...", foreground="blue"))
                        except socket.timeout:
                            break
            except Exception as e:
                self.root.after(0, lambda: self.message_label.config(
                    text=f"Erro na calibração: {str(e)}", foreground="red"))

        threading.Thread(target=run_calibration, daemon=True).start()

    def selecionar_tipo_arquivo(self, tipo):
        """Inicia scan com o tipo de arquivo selecionado"""
        self.file_type.set(tipo)
        self.file_options_frame.pack_forget()
        self.save_btn.config(state=tk.DISABLED)
        self.log_area.delete(1.0, tk.END)

        # Solicitar altura do objeto
        height_dialog = tk.Toplevel(self.root)
        height_dialog.title("Altura do Objeto")
        height_dialog.geometry("300x150")
        height_dialog.transient(self.root)
        height_dialog.resizable(False, False)
        height_dialog.grab_set()

        ttk.Label(height_dialog, text="Introduza a altura do objeto em cm:").pack(pady=10)

        height_var = tk.StringVar(value="10.0")
        height_entry = ttk.Entry(height_dialog, textvariable=height_var, width=10)
        height_entry.pack(pady=10)
        height_entry.select_range(0, tk.END)
        height_entry.focus_set()

        def on_ok():
            try:
                altura = float(height_var.get())
                if altura <= 0 or altura > 26:
                    messagebox.showerror("Erro", "A altura deve ser maior que 0 e menor ou igual a 26 cm!")
                    return

                height_dialog.destroy()
                self.iniciar_scan_com_altura(tipo, altura)
            except ValueError:
                messagebox.showerror("Erro", "Por favor, digite um valor numérico válido!")

        def on_cancel():
            height_dialog.destroy()

        btn_frame = ttk.Frame(height_dialog)
        btn_frame.pack(pady=10, fill=tk.X)

        ttk.Button(btn_frame, text="OK", command=on_ok).pack(side=tk.LEFT, padx=20, expand=True)
        ttk.Button(btn_frame, text="Cancelar", command=on_cancel).pack(side=tk.RIGHT, padx=20, expand=True)

        height_dialog.bind('<Return>', lambda e: on_ok())

    def iniciar_scan_com_altura(self, tipo, altura):
        """Inicia scan com tipo de arquivo e altura especificados"""
        comando = f"iniciar_{tipo}|{altura}"
        if self.fast_scan.get():
            comando = f"iniciar_{tipo}_fast|{altura}"

        self.log_area.insert(tk.END, f"Altura máxima definida: {altura} cm\n\n")

        self.scanning = True
        self.iniciar_btn.config(text="Cancelar")
        threading.Thread(target=self.receber_dados_scan, args=(comando,), daemon=True).start()

    def receber_dados_scan(self, comando):
        """Recebe dados continuamente do scan"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.settimeout(10)
                s.connect((self.arduino_ip, self.arduino_port))

                # Envia comando
                s.sendall(comando.encode('utf-8'))
                self.root.after(0, lambda: self.message_label.config(
                    text="Scanning...", foreground="blue"))

                buffer = ""
                while self.scanning:
                    try:
                        s.settimeout(3)
                        data = s.recv(1024).decode('utf-8')
                        if not data:
                            break

                        buffer += data

                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            if line := line.strip():
                                # Atualizar log
                                self.root.after(0, lambda l=line:
                                self.log_area.insert(tk.END, l + "\n"))
                                self.root.after(0, lambda:
                                self.log_area.yview(tk.END))

                                # Verificar fim do scan
                                if any(x in line for x in ["SCAN_COMPLETE", "SCAN_FINALIZADO", "SCAN_CANCELADO"]):
                                    self.root.after(0, lambda l=line: self.finalizar_scan(l))
                                    return
                    except socket.timeout:
                        if not self.scanning:
                            break
        except Exception as e:
            self.root.after(0, lambda: self.finalizar_scan(f"ERRO: {str(e)}"))

    def finalizar_scan(self, mensagem=None):
        """Finaliza o scan e prepara processamento"""
        self.scanning = False
        self.iniciar_btn.config(text="Iniciar")

        if mensagem and "SCAN_CANCELADO" in mensagem:
            self.message_label.config(text="Scan cancelado", foreground="orange")
        elif mensagem and "ERRO" in mensagem:
            self.message_label.config(text=mensagem, foreground="red")
        else:
            self.message_label.config(text="Scan concluído com sucesso", foreground="green")

        if self.log_area.get(1.0, tk.END).strip():
            self.raw_points = self.extract_points_from_log()
            self.save_btn.config(state=tk.NORMAL)
            for btn in self.proc_buttons:
                btn.config(state=tk.NORMAL)

    def extract_points_from_log(self):
        """Extrai pontos XYZ do log"""
        pontos_xyz = []
        for linha in self.log_area.get(1.0, tk.END).strip().splitlines():
            linha = linha.strip()

            if linha.startswith("RAW_DATA|"):
                parts = linha.split('|')
                if len(parts) == 5:
                    try:
                        angle = float(parts[1])
                        distance = float(parts[2])
                        height = float(parts[3])
                        offset = float(parts[4])

                        # Converter para XYZ
                        angle_rad = math.radians(angle)
                        corrected_distance = offset - distance
                        x = corrected_distance * math.cos(angle_rad)
                        y = corrected_distance * math.sin(angle_rad)
                        z = height

                        pontos_xyz.append([x, y, z])
                    except ValueError:
                        continue
            # Formato x y z direto
            elif re.match(r"^\s*-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s+-?\d+(\.\d+)?\s*$", linha):
                try:
                    pontos_xyz.append(list(map(float, linha.split())))
                except ValueError:
                    continue

        return pontos_xyz

    def guardar_dados(self):
        """Guarda dados em arquivo TXT ou STL"""
        pontos = self.processed_points if len(self.processed_points) > 0 else self.raw_points

        if not pontos:
            messagebox.showwarning("Sem Dados", "Nenhum dado disponível para guardar.")
            return

        agora = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        file_type = self.file_type.get()

        if file_type == "txt":
            file_path = filedialog.asksaveasfilename(
                defaultextension=".txt",
                filetypes=[("Arquivo TXT", "*.txt")],
                initialfile=f"scan_{agora}.txt")

            if file_path:
                with open(file_path, "w", encoding="utf-8") as f:
                    for p in pontos:
                        f.write(f"{p[0]} {p[1]} {p[2]}\n")
                messagebox.showinfo("Guardado", f"Arquivo TXT guardado em:\n{file_path}")

        elif file_type == "stl":
            file_path = filedialog.asksaveasfilename(
                defaultextension=".stl",
                filetypes=[("Arquivo STL", "*.stl")],
                initialfile=f"scan_{agora}.stl")

            if file_path:
                try:
                    mesh, _ = self.process_scan_to_solid(
                        self.processed_points if len(self.processed_points) > 0 else np.array(self.raw_points))
                    o3d.io.write_triangle_mesh(file_path, mesh)
                    messagebox.showinfo("Guardado", f"Arquivo STL guardado em:\n{file_path}")
                except Exception as e:
                    messagebox.showerror("Erro", f"Erro ao guardar STL: {str(e)}")

    def create_embedded_visualization(self):
        """Visualiza pontos na interface"""
        for widget in self.viz_frame.winfo_children():
            widget.destroy()

        points = np.array(self.processed_points if len(self.processed_points) > 0 else self.raw_points)

        if len(points) == 0:
            messagebox.showinfo("Sem dados", "Não há pontos para visualizar.")
            return

        fig = Figure(figsize=(8, 6), dpi=100)
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=2, c='blue', alpha=0.7)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Nuvem de Pontos 3D')

        canvas = FigureCanvasTkAgg(fig, master=self.viz_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        NavigationToolbar2Tk(canvas, self.viz_frame).pack(fill=tk.X)

    def process_point_cloud(self):
        """Processa pontos para melhorar qualidade"""
        if not self.raw_points:
            messagebox.showinfo("Sem dados", "Não há pontos para processar.")
            return

        progress = tk.Toplevel(self.root)
        progress.title("A processar...")
        progress.geometry("400x100")
        progress.transient(self.root)

        label = ttk.Label(progress, text="A processar nuvem de pontos...")
        label.pack(pady=10)

        bar = ttk.Progressbar(progress, orient=tk.HORIZONTAL, length=350, mode='determinate')
        bar.pack(pady=10)

        def process_thread():
            try:
                self.root.after(0, lambda: bar.config(value=20))
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array(self.raw_points))

                self.root.after(0, lambda: label.config(text="A remover ruídos..."))
                self.root.after(0, lambda: bar.config(value=40))
                processed_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.5)

                self.root.after(0, lambda: label.config(text="A regularizar a distribuição..."))
                self.root.after(0, lambda: bar.config(value=60))

                processed_pcd = processed_pcd.voxel_down_sample(voxel_size=0.05)

                self.root.after(0, lambda: label.config(text="A finalizar..."))
                self.root.after(0, lambda: bar.config(value=80))
                processed_pcd.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

                self.processed_points = np.asarray(processed_pcd.points)

                self.root.after(0, lambda: self.log_area.insert(tk.END,
                                                                "\n\n--- PROCESSAMENTO CONCLUÍDO ---\n"))
                self.root.after(0, lambda: self.log_area.insert(tk.END,
                                                                f"Pontos originais: {len(self.raw_points)}\n"))
                self.root.after(0, lambda: self.log_area.insert(tk.END,
                                                                f"Pontos processados: {len(self.processed_points)}\n"))

                self.root.after(0, lambda: bar.config(value=100))
                self.root.after(1000, progress.destroy)
                self.root.after(1500, self.visualize_comparison)

            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Erro", f"Erro: {str(e)}"))
                self.root.after(0, progress.destroy)

        threading.Thread(target=process_thread, daemon=True).start()

    def visualize_comparison(self):
        """Visualiza comparação entre original e processado"""
        for widget in self.viz_frame.winfo_children():
            widget.destroy()

        if len(self.raw_points) == 0 or len(self.processed_points) == 0:
            return

        fig = Figure(figsize=(12, 6), dpi=100)

        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(np.array(self.raw_points)[:, 0],
                    np.array(self.raw_points)[:, 1],
                    np.array(self.raw_points)[:, 2],
                    s=2, c='blue', alpha=0.7)
        ax1.set_title('Pontos Originais')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')

        ax2 = fig.add_subplot(122, projection='3d')
        ax2.scatter(self.processed_points[:, 0],
                    self.processed_points[:, 1],
                    self.processed_points[:, 2],
                    s=2, c='red', alpha=0.7)
        ax2.set_title('Pontos Processados')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')

        canvas = FigureCanvasTkAgg(fig, master=self.viz_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        NavigationToolbar2Tk(canvas, self.viz_frame).pack(fill=tk.X)

    def generate_solid_model(self):
        """Gera modelo sólido 3D"""
        points = self.processed_points if len(self.processed_points) > 0 else np.array(self.raw_points)

        if len(points) < 100:
            messagebox.showwarning("Pontos Insuficientes",
                                   "São necessários pelo menos 100 pontos para gerar um modelo sólido.")
            return

        progress = tk.Toplevel(self.root)
        progress.title("A gerar um Modelo Sólido")
        progress.geometry("400x120")
        progress.transient(self.root)

        label = ttk.Label(progress, text="A iniciar criação de sólido...")
        label.pack(pady=10)

        bar = ttk.Progressbar(progress, orient=tk.HORIZONTAL, length=350, mode='indeterminate')
        bar.pack(pady=10)
        bar.start()

        log_text = ttk.Label(progress, text="", wraplength=380)
        log_text.pack(pady=5, fill='x')

        def process_thread():
            try:
                self.root.after(0, lambda: log_text.config(text="A processar..."))
                mesh, vertices = self.process_scan_to_solid(points)

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                file_path = filedialog.asksaveasfilename(
                    defaultextension=".stl",
                    filetypes=[("Arquivo STL", "*.stl")],
                    initialfile=f"scan_solid_{timestamp}.stl")

                if file_path:
                    o3d.io.write_triangle_mesh(file_path, mesh)

                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(vertices)

                    self.root.after(0, lambda: messagebox.showinfo(
                        "Sucesso", f"Modelo guardado em:\n{file_path}"))
                    self.root.after(100, lambda: self.visualize_mesh(mesh, pcd))

                self.root.after(0, progress.destroy)
            except Exception as e:
                import traceback
                print("=== ERRO DETALHADO ===")
                print(traceback.format_exc())

                self.root.after(0, lambda: messagebox.showerror(
                    "Erro", f"Erro: {str(e)}"))
                self.root.after(0, progress.destroy)

        threading.Thread(target=process_thread, daemon=True).start()

    def visualize_mesh(self, mesh, pcd):
        """Visualiza malha 3D"""
        for widget in self.viz_frame.winfo_children():
            widget.destroy()

        fig = plt.figure(figsize=(12, 6))

        ax1 = fig.add_subplot(121, projection='3d')
        points = np.asarray(pcd.points)
        ax1.scatter(points[:, 0], points[:, 1], points[:, 2], s=2, c='blue', alpha=0.5)
        ax1.set_title('Nuvem de Pontos')

        ax2 = fig.add_subplot(122, projection='3d')
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)

        tri_mesh = ax2.plot_trisurf(
            vertices[:, 0],
            vertices[:, 1],
            vertices[:, 2],
            triangles=triangles,
            color='lightblue',
            shade=True,
            alpha=0.9)

        ax2.set_title('Modelo Sólido')

        for ax in [ax1, ax2]:
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.grid(True)

            ax.set_xlim([np.min(vertices[:, 0]), np.max(vertices[:, 0])])
            ax.set_ylim([np.min(vertices[:, 1]), np.max(vertices[:, 1])])
            ax.set_zlim([np.min(vertices[:, 2]), np.max(vertices[:, 2])])

            ax.view_init(elev=30, azim=45)

        plt.tight_layout()

        canvas = FigureCanvasTkAgg(fig, master=self.viz_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        NavigationToolbar2Tk(canvas, self.viz_frame).pack(fill=tk.X)

        ttk.Button(self.viz_frame, text="Visualizar em 3D Interativo",
                   command=lambda: self.show_interactive_3d(mesh)).pack(pady=5)

    def show_interactive_3d(self, mesh):
        """Abre visualização 3D interativa"""

        def view_thread():
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name="Modelo 3D", width=800, height=600)

            mesh.paint_uniform_color([0.7, 0.7, 0.9])
            vis.add_geometry(mesh)

            view_control = vis.get_view_control()
            view_control.set_zoom(0.8)

            opt = vis.get_render_option()
            opt.mesh_show_wireframe = False
            opt.mesh_shade_option = o3d.visualization.MeshShadeOption.Color

            vis.run()
            vis.destroy_window()

        threading.Thread(target=view_thread, daemon=True).start()

    def process_scan_to_solid(self, points):
        """Gera malha sólida a partir de pontos"""
        if not isinstance(points, np.ndarray):
            points = np.array(points)

        window_size = 5
        smooth_iterations = 2
        max_distance, min_distance = 20, 0

        # Coordenadas cilíndricas
        xy = points[:, :2]
        r = np.sqrt(np.sum(xy ** 2, axis=1))
        theta = np.arctan2(points[:, 1], points[:, 0])
        theta[theta < 0] += 2 * np.pi
        z = points[:, 2]

        num_angles = 360
        z_min, z_max = np.min(z), np.max(z)
        z_delta = max((z_max - z_min) / 60, 0.03)
        num_heights = max(int((z_max - z_min) / z_delta) + 1, 15)

        r_matrix = np.full((num_heights, num_angles), np.nan)
        theta_matrix = np.zeros((num_heights, num_angles))
        z_matrix = np.zeros((num_heights, num_angles))

        for i in range(len(points)):
            height_idx = min(int((z[i] - z_min) / z_delta), num_heights - 1)
            angle_idx = min(int(theta[i] * num_angles / (2 * np.pi)), num_angles - 1)
            r_matrix[height_idx, angle_idx] = r[i]

        r_matrix[r_matrix > max_distance] = np.nan
        r_matrix[r_matrix < min_distance] = np.nan

        for i in range(num_heights):
            for j in range(num_angles):
                theta_matrix[i, j] = j * 2 * np.pi / num_angles
                z_matrix[i, j] = z_min + i * z_delta

        # Interpolar valores em falta
        for i in range(num_heights):
            valid_indices = np.where(~np.isnan(r_matrix[i, :]))[0]
            if len(valid_indices) == 0:
                # Usar camadas adjacentes
                valid_heights = []
                if i > 0 and np.any(~np.isnan(r_matrix[i - 1, :])):
                    valid_heights.append(i - 1)
                if i < num_heights - 1 and np.any(~np.isnan(r_matrix[i + 1, :])):
                    valid_heights.append(i + 1)

                if valid_heights:
                    r_matrix[i, :] = np.nanmean([r_matrix[h, :] for h in valid_heights], axis=0)
                continue

            # Interpolar valores circulares
            for j in range(num_angles):
                if np.isnan(r_matrix[i, j]):
                    left_idx = j
                    while left_idx >= 0 and np.isnan(r_matrix[i, left_idx]):
                        left_idx -= 1
                    if left_idx < 0:
                        left_idx = num_angles - 1
                        while left_idx >= 0 and np.isnan(r_matrix[i, left_idx]):
                            left_idx -= 1

                    right_idx = j
                    while right_idx < num_angles and np.isnan(r_matrix[i, right_idx]):
                        right_idx += 1
                    if right_idx >= num_angles:
                        right_idx = 0
                        while right_idx < num_angles and np.isnan(r_matrix[i, right_idx]):
                            right_idx += 1

                    if left_idx >= 0 and right_idx < num_angles:
                        left_val = r_matrix[i, left_idx]
                        right_val = r_matrix[i, right_idx]

                        if right_idx < left_idx:
                            right_idx += num_angles

                        dist_left = (j - left_idx) % num_angles
                        dist_right = (right_idx - j) % num_angles
                        total_dist = dist_left + dist_right

                        if total_dist > 0:
                            r_matrix[i, j] = (right_val * dist_left + left_val * dist_right) / total_dist

        for i in range(num_heights):
            row_mean = np.nanmean(r_matrix[i, :])
            if np.isfinite(row_mean):
                r_matrix[i, np.isnan(r_matrix[i, :])] = row_mean

        r_smoothed = r_matrix.copy()

        for _ in range(smooth_iterations):
            pad_width = window_size // 2
            r_padded = np.pad(r_smoothed, ((0, 0), (pad_width, pad_width)), mode='wrap')
            kernel = np.ones((1, window_size)) / window_size
            r_smoothed = ndimage.convolve(r_padded, kernel, mode='constant')
            r_smoothed = r_smoothed[:, pad_width:-pad_width]

        vertical_kernel_size = 3  # Reduzido para não perder detalhes
        pad_height = vertical_kernel_size // 2
        r_padded_v = np.pad(r_smoothed, ((pad_height, pad_height), (0, 0)), mode='reflect')
        vertical_kernel = np.ones((vertical_kernel_size, 1)) / vertical_kernel_size
        r_smoothed = ndimage.convolve(r_padded_v, vertical_kernel, mode='constant')
        r_smoothed = r_smoothed[pad_height:-pad_height, :]

        r_smoothed = np.column_stack((r_smoothed, r_smoothed[:, 0]))
        theta_matrix = np.column_stack((theta_matrix, theta_matrix[:, 0] + 2 * np.pi))
        z_matrix = np.column_stack((z_matrix, z_matrix[:, 0]))
        num_angles += 1

        x_matrix = r_smoothed * np.cos(theta_matrix)
        y_matrix = r_smoothed * np.sin(theta_matrix)

        x_top = np.nanmean(x_matrix[-1, :])
        y_top = np.nanmean(y_matrix[-1, :])
        z_top = z_matrix[-1, 0] + (z_matrix[-1, 0] - z_matrix[-2, 0])

        x_bottom = np.nanmean(x_matrix[0, :])
        y_bottom = np.nanmean(y_matrix[0, :])
        z_bottom = z_matrix[0, 0] - (z_matrix[1, 0] - z_matrix[0, 0])

        vertices = []
        for i in range(num_heights):
            for j in range(num_angles):
                vertices.append([x_matrix[i, j], y_matrix[i, j], z_matrix[i, j]])

        top_idx = len(vertices)
        vertices.append([x_top, y_top, z_top])

        bottom_idx = len(vertices)
        vertices.append([x_bottom, y_bottom, z_bottom])

        triangles = []

        for i in range(num_heights - 1):
            for j in range(num_angles - 1):
                v1 = i * num_angles + j
                v2 = i * num_angles + j + 1
                v3 = (i + 1) * num_angles + j
                v4 = (i + 1) * num_angles + j + 1

                triangles.append([v1, v3, v2])
                triangles.append([v2, v3, v4])

        for i in range(num_heights - 1):
            v1 = i * num_angles + (num_angles - 1)
            v2 = i * num_angles
            v3 = (i + 1) * num_angles + (num_angles - 1)
            v4 = (i + 1) * num_angles

            triangles.append([v1, v3, v2])
            triangles.append([v2, v3, v4])

        for j in range(num_angles - 1):
            v1 = (num_heights - 1) * num_angles + j
            v2 = (num_heights - 1) * num_angles + j + 1
            triangles.append([v1, v2, top_idx])
        triangles.append([
            (num_heights - 1) * num_angles + (num_angles - 1),
            (num_heights - 1) * num_angles,
            top_idx
        ])

        for j in range(num_angles - 1):
            triangles.append([j, bottom_idx, j + 1])
        triangles.append([num_angles - 1, bottom_idx, 0])

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(np.array(vertices))
        mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))

        mesh = mesh.filter_smooth_simple(number_of_iterations=3)  # Reduzido para 3 iterações

        mesh.compute_vertex_normals()
        mesh.orient_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_degenerate_triangles()

        return mesh, np.array(vertices)


if __name__ == "__main__":
    root = tk.Tk()
    app = Scanner3D(root)
    root.mainloop()