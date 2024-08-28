### Panduan Singkat Instalasi dan Penggunaan

#### 1. Instalasi ROS Melodic (Lebih lanjut dan Lengkap ada di 'setup.md')
1. **Tambahkan Repository ROS**: Tambahkan repository ROS Melodic ke sistem Ubuntu Anda.
2. **Instal ROS Melodic Desktop Full**: Instal versi lengkap ROS Melodic.
3. **Inisialisasi rosdep**: Inisialisasi dan perbarui rosdep untuk mengelola dependensi ROS.
4. **Pengaturan Lingkungan ROS**: Tambahkan ROS ke `~/.bashrc` agar otomatis dimuat pada setiap sesi.
5. **Instalasi Dependensi ROS**: Instal alat bantu dan dependensi yang diperlukan.

#### 2. Menyiapkan Workspace ROS
1. **Buat Workspace**: Buat workspace baru di `~/catkin_ws`.
2. **Sumberkan Workspace**: Tambahkan workspace ke sesi bash untuk mengenali paket ROS.

#### 3. Clone dan Build Paket Magician ROS
1. **Clone Repository**: Clone repository Magician ROS ke `~/catkin_ws/src`.
2. **Instal Dependensi**: Instal semua dependensi yang diperlukan untuk paket.
3. **Build Workspace**: Build paket di workspace dengan `catkin_make`.

#### 4. Instalasi Driver CH343
1. **Download dan Instal Driver**: Download, build, dan instal driver CH343 untuk Linux.
2. **Verifikasi Instalasi**: Verifikasi instalasi driver dengan `dmesg` dan cek perangkat serial.

#### 5. Menjalankan Dobot Server
1. **Jalankan ROS**: Mulai `roscore` dan `rosrun dobot DobotServer` sesuai port yang digunakan.

#### 6. Tambahkan Rospkg dan Tkinter
1. **Instal Rospkg**: Instal pustaka `rospkg` untuk interaksi ROS dengan Python.
2. **Instal Tkinter**: Instal Tkinter untuk mendukung GUI di Python.

#### 7. Menjalankan `PythonUI.py`
1. **Jalankan GUI**: Jalankan `PythonUI.py` untuk berinteraksi dengan Dobot.

### SOP Menjalankan `main.py`
1. **Pastikan Semua Terpasang**: Pastikan `ref1.jpg` dan `ref2.jpg`, serta video capture sudah diatur ke webcam.
2. **Jalankan ROS**: Pastikan `roscore` dan `DobotServer` sudah berjalan.
3. **Jalankan `main.py`**: Jalankan `main.py` dan tekan tombol "i" untuk memulai.
4. **Ulangi Deteksi**: Tekan "r" untuk mengulang deteksi jika diperlukan.
5. **Keluar**: Tekan "q" untuk keluar dari program.

Kode yang digunakan (`main.py`) mengatur Dobot untuk mengenali objek dengan ORB, melakukan homography, dan melakukan proses pick-and-place berdasarkan klasifikasi objek menggunakan referensi gambar.
