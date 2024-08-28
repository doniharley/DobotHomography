# Dobot Homography

Berikut adalah panduan lengkap mencakup semua langkah untuk menyiapkan ROS Melodic, instalasi driver, dan menjalankan proyek Dobot dengan RViz dan PythonUI.

---

## 1. Instalasi ROS Melodic

Langkah pertama dalam proyek ini adalah menginstal ROS Melodic di sistem Ubuntu 18.04 AtauROS Kinetic Ubuntu 16.04 Anda. Ikuti langkah-langkah berikut:

### 1.1. Tambahkan Repository ROS Melodic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

### 1.2. Instal ROS Melodic Desktop Full

```bash
sudo apt install ros-melodic-desktop-full
```

### 1.3. Inisialisasi rosdep

```bash
sudo rosdep init
rosdep update
```

### 1.4. Pengaturan Lingkungan ROS

Agar ROS secara otomatis dimuat di setiap sesi bash, tambahkan perintah berikut ke dalam `.bashrc`:

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.5. Instalasi Dependensi ROS

Untuk memastikan semua dependensi yang diperlukan untuk membangun paket ROS terinstal:

```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## 2. Membuat dan Menyiapkan Workspace ROS

### 2.1. Membuat Workspace

Pertama, buat workspace baru untuk ROS:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### 2.2. Sumberkan Workspace

Tambahkan workspace ini ke sesi bash Anda agar ROS dapat mengenali paket-paket di dalamnya:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Clone dan Build Paket Magician ROS

### 3.1. Clone Repository Magician ROS

Navigasi ke direktori `src` dalam workspace Anda dan clone repository:

```bash
cd ~/catkin_ws/src
git clone https://github.com/Dobot-Arm/Magician_ROS.git
```

### 3.2. Instal Dependensi

Instal semua dependensi yang diperlukan untuk paket:

```bash
rosdep install --from-paths #. --ignore-src -r -y
```

### 3.3. Build Workspace

Setelah menginstal dependensi, kembali ke direktori workspace dan build paket:

```bash
cd ~/catkin_ws
catkin_make
```

## 4. (Opsional) Clone dan Siapkan Paket RViz

### 4.1. Clone Repository RViz

Tetap di dalam direktori `src`, clone repository RViz untuk Dobot:

```bash
cd ~/catkin_ws/src
git clone https://github.com/nj-ramadhan/polman-dsk-ros-arm-robot-dobot.git
```

### 4.2. Build Workspace Kembali

Setelah meng-clone paket RViz, build workspace Anda sekali lagi:

```bash
cd ~/catkin_ws
catkin_make
```
### 4.3 Instal Python wxPython
Pastikan folder msg di src ada

```bash
mkdir ~/src/msg
```

Jika Anda menggunakan Python 3, Anda mungkin perlu menginstalnya menggunakan pip:

```bash
pip install wxPython
```

### 4.3. Buat Script Menjadi Eksekutabel

Agar semua skrip yang diperlukan dapat dijalankan, pastikan skrip tersebut dapat dieksekusi dengan perintah berikut:

```bash
cd ~/catkin_ws/src/dobot/scripts
chmod +x *dobot_state_publisherX
		  dobot_move_publisherX*
```

Perintah ini akan membuat hampir semua skrip dalam direktori `/scripts` dapat dieksekusi.

### 4.4. Menjalankan Visualisasi di RViz

Setelah Menginstall dan menjalankan langkah 8, RViz Untuk memvisualisasikan Dobot di RViz, jalankan perintah berikut:

```bash
roslaunch dobot rviz.launch
```

Perintah ini akan meluncurkan visualisasi RViz untuk Dobot.
## 5. Menemukan Alamat Port untuk Dobot

Sebelum menjalankan server Dobot, Anda perlu menemukan alamat port di mana Dobot terhubung. Ini biasanya adalah `ttyACM0`. Gunakan perintah berikut untuk menemukan port:

```bash
ls /dev -l
```

Cari perangkat yang berlabel `ttyACM0` atau port lainnya yang sesuai dengan perangkat Dobot Anda.

Tetapi Disarankan untuk menginstal ch343
## 6. Instalasi Driver CH343 di Linux

### 6.1. Download Driver CH343

Untuk mempermudah pemakaian Untuk mendownload driver CH343, gunakan perintah berikut:

```bash
cd ~/Downloads
git clone https://github.com/WCHSoftGroup/ch343ser_linux.git
```

### 6.2. Build dan Instal Driver

Masuk ke direktori driver yang telah didownload, kemudian compile driver:

```bash
cd ch343ser_linux
make
```

Setelah driver berhasil dikompilasi, instal dengan perintah:

```bash
sudo make load
```

### 6.3. Verifikasi Instalasi

Untuk memastikan bahwa perangkat telah dikenali oleh sistem, Anda bisa menggunakan perintah berikut:

```bash
dmesg | grep tty
```

Kemudian, cek apakah perangkat serial tersedia dengan:

```bash
ls /dev/ttyCH*
```

Jika perlu, Anda bisa mengatur izin akses ke perangkat:

```bash
sudo chmod 666 /dev/ttyCH343USB0
```

Gantilah `/dev/ttyCH343USB0` dengan nama perangkat yang sesuai jika berbeda.



## 7. Menjalankan Dobot Server

Setelah semua diatur, mulailah `roscore` dan node `DobotServer` dengan perintah berikut:

```bash
roscore &
rosrun dobot DobotServer /dev/ttyACM0
```

Gantilah `/dev/ttyACM0` dengan port yang ditemukan sebelumnya jika berbeda.

Atau Setelah Installai ch343

```bash
roscore &
rosrun dobot DobotServer /dev/ttyCH343USB0
```

## 8. Menambahkan Rospkg atau RosPy dan tkinter

Sebelum menjalankan `main.py`, pastikan Anda telah menambahkan pustaka `ropkg` atau `rospy`, yang diperlukan untuk menggunakan pustaka ROS di Python. Instal pustaka ini dengan perintah berikut:

Untuk `rospkg`:

```bash
pip install rospkg
```

Untuk `rospy`, biasanya sudah termasuk dalam instalasi ROS, tetapi jika Anda memerlukannya secara terpisah:

```bash
sudo apt-get install python-rospy
```

tkinter adalah library untuk mejalankan main.py dan PythonUI2.py

```bash
sudo apt-get install python3-tk
```

atau untuk python2

```bash
sudo apt-get install python-tk
```
## 9. Menjalankan `PythonUI.py`

Terakhir, navigasikan ke direktori di mana `PythonUI.py` berada dan jalankan dengan perintah berikut:

```bash
cd /path/to/your/python/script
python PythonUI.py
```

Gantilah `/path/to/your/python/script` dengan jalur aktual di mana `PythonUI.py` berada.

**![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXc8-yX4_WMW13iEItzV9yKYxzHLSN-jMQI3uauDUc1vFimsGcwwdpoDP694IVotBz0QmPKJSHllFTBc1LgSD4kJqq-eSYg2tCQE7EcRxJqHA509bVZnQv3s-FvEned9GUsNXOPJVCWMi7-4zCkPnGrDF6o?key=xi09SMKGetAa6D1tbP737A)**
**![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXf76d7v-m7ZBAoPoh95CJ8HLg9gouPZXrMqWHEADKIet0G9AK1oG3ENvvHsW2R2HluDpW_o_NTOI_LSTNdH39NxoGQHtbSCPbe5uReQk5y5Ss3FmoSnLNaG79mRXZjEVlfBTqgsLKrCkQ-U2pViJVBnr_B4?key=xi09SMKGetAa6D1tbP737A)

**
tampilan PythonUI di linux dan berikut fitur-fitur yang ada:

- **SetPTPParams** untuk mengatur akselerasi dan kecepatan gerakan lengan robot.
- **JumpParams** untuk mengatur gerakan lompat atas-bawah robot; setel ke 0 untuk pengaturan default.
- **Run DobotServer** untuk menjalankan server utama Dobot Magician.
- **Activate/Deactivate Suction Cup** untuk menyalakan/mematikan suction cup (pastikan alat sudah terpasang dengan benar).
- **Clear Alarm Status** untuk menghapus alarm atau menghilangkan indikasi lampu LED merah (untuk informasi lebih lanjut mengenai alarm, lihat di Dobot Magician API).
- **Home Position** untuk mengkalibrasi posisi robot jika tidak akurat; pastikan area kerja bersih untuk menghindari tabrakan.
- **Watch Get Pose** untuk memonitor setiap gerakan koordinat robot setiap 1,8 detik.

## 10. Menjalankan Program Utama

Sekarang, setelah semuanya siap, Anda bisa mulai menggunakan program utama dengan berinteraksi melalui UI yang disediakan oleh `PythonUI.py`.

SOP Menjalankan main.py
- Pastikan Semua Terpasang: Pastikan ref1.jpg dan ref2.jpg, serta video capture sudah diatur ke webcam.
- Jalankan ROS: Pastikan roscore dan DobotServer sudah berjalan.
- Jalankan main.py: Jalankan main.py dan lalu posisikan robot dengan koordinat berikut command nya,
```bash
rosservice call /DobotServer/SetPTPCmd "{ptpMode: 0, x: 0, y: 110, z: 0, r: 0}"
````
- pastikan tegak lurus dan sejajar dengan center point di kamera setelah itu,
- tekan tombol "i" untuk memulai menekan 4 titik untuk menerapkan homography pada box.
- Ulangi Deteksi: Tekan "r" untuk mengulang deteksi jika diperlukan.
- Jika sudah tekan tombol execute dalam window tkinter "Homography and Coordinates Display" untuk mengeksekusi
- Keluar: Tekan "q" untuk keluar dari program.

---

mulai dari instalasi ROS Melodic, instalasi driver CH343, hingga menjalankan Dobot Magician dengan RViz dan PythonUI. Setiap langkah dijelaskan secara rinci untuk memastikan lingkungan Anda dikonfigurasi dengan benar dan semua perangkat serta perangkat lunak berfungsi sebagaimana mestinya.

Keterangan, PythonUI dan main.py menggunakan python 3.9 dengan pyenv di linux ubuntu 

dan informasi lebih lanjut bisa ke; https://download.dobot.cc/development-protocol/dobot-magician/win7-win10/en/Dobot-Magician-ROS-Demo.zip 

