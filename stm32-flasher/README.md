# stm32-flasher — UI untuk upload firmware ke STM32

Aplikasi lokal kecil untuk build dan menulis firmware ke board STM32 lewat ST-LINK,
tanpa perlu mengetik perintah. Backend Python (pustaka standar saja), UI dibuka di browser.

```sh
./run.sh          # atau: python3 server.py
```

Browser terbuka sendiri di <http://127.0.0.1:8731>. Hentikan dengan `Ctrl+C`.

## Isi layar

| Bagian | Fungsi |
|---|---|
| Pil status di kanan atas | Hasil `st-info --probe` tiap 2,5 detik — tipe chip, ukuran flash. Arahkan kursor untuk chipid, serial, versi ST-LINK. |
| **Dari folder proyek** | Jalankan `make`, `make flash`, `make clean` di folder mana pun yang punya Makefile. Daftar `.bin` di bawahnya punya tombol Flash sendiri-sendiri. |
| **Dari file .bin** | Tarik-lepas atau pilih file, tentukan alamat, tulis ke board. Berguna untuk firmware yang dibuild di tempat lain. |
| **Backup & pemulihan** | `make backup`, dump seluruh isi flash ke `backup/board-dump.bin`, `make restore`, dan erase penuh (dengan konfirmasi). |
| Panel log | Keluaran perintah mengalir langsung, sama persis dengan yang muncul di terminal. |

Folder proyek terakhir diingat di `localStorage`; nilai awalnya `stm32-pd`.

## Syarat

```sh
brew install arm-none-eabi-gcc stlink
```

Tool yang belum terpasang dilaporkan di log saat start dan lewat notifikasi di UI.

## Catatan

- Server hanya mengikat `127.0.0.1`, tidak terjangkau dari jaringan.
- Perintah dibangun dari daftar tetap (`MAKE_ACTIONS` di `server.py`) tanpa shell, jadi
  isian dari browser tidak bisa menjadi perintah. Alamat flash wajib cocok pola hex,
  nama file unggahan disanitasi ke `uploads/`.
- Hanya satu proses boleh jalan sekaligus; tombol terkunci selama proses berlangsung.
- Probe board dijeda selama flashing supaya `st-info` tidak berebut ST-LINK.
- Isi `uploads/` tidak masuk git.
