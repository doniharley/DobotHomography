1. Instalasi ROS Melodic
Tambahkan Repository ROS: Tambahkan repository ROS Melodic ke sistem Ubuntu Anda.
Instal ROS Melodic Desktop Full: Instal versi lengkap ROS Melodic.
Inisialisasi rosdep: Inisialisasi dan perbarui rosdep untuk mengelola dependensi ROS.
Pengaturan Lingkungan ROS: Tambahkan ROS ke ~/.bashrc agar otomatis dimuat pada setiap sesi.
Instalasi Dependensi ROS: Instal alat bantu dan dependensi yang diperlukan.
2. Menyiapkan Workspace ROS
Buat Workspace: Buat workspace baru di ~/catkin_ws.
Sumberkan Workspace: Tambahkan workspace ke sesi bash untuk mengenali paket ROS.
3. Clone dan Build Paket Magician ROS
Clone Repository: Clone repository Magician ROS ke ~/catkin_ws/src.
Instal Dependensi: Instal semua dependensi yang diperlukan untuk paket.
Build Workspace: Build paket di workspace dengan catkin_make.
4. Instalasi Driver CH343
Download dan Instal Driver: Download, build, dan instal driver CH343 untuk Linux.
Verifikasi Instalasi: Verifikasi instalasi driver dengan dmesg dan cek perangkat serial.
5. Menjalankan Dobot Server
Jalankan ROS: Mulai roscore dan rosrun dobot DobotServer sesuai port yang digunakan.
6. Tambahkan Rospkg dan Tkinter
Instal Rospkg: Instal pustaka rospkg untuk interaksi ROS dengan Python.
Instal Tkinter: Instal Tkinter untuk mendukung GUI di Python.
7. Menjalankan PythonUI.py
Jalankan GUI: Jalankan PythonUI.py untuk berinteraksi dengan Dobot.
SOP Menjalankan main.py
Pastikan Semua Terpasang: Pastikan ref1.jpg dan ref2.jpg, serta video capture sudah diatur ke webcam.
Jalankan ROS: Pastikan roscore dan DobotServer sudah berjalan.
Jalankan main.py: Jalankan main.py dan tekan tombol "i" untuk memulai.
Ulangi Deteksi: Tekan "r" untuk mengulang deteksi jika diperlukan.
Keluar: Tekan "q" untuk keluar dari program.
