use std::{
    io::Write,
    os::unix::net::UnixListener,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

use crossbeam_channel::{Receiver, TryRecvError};

pub fn spawn_lidar_thread(rx: Receiver<Vec<u8>>, connected: Arc<AtomicBool>) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("haarper_lidar".into())
        .spawn(move || {
            let path = "/tmp/haarper_lidar.sock";
            let _ = std::fs::remove_file(path);
            let listener = UnixListener::bind(path).expect("bind lidar socket");

            loop {
                connected.store(false, Ordering::Relaxed);
                let Ok((mut stream, _)) = listener.accept() else {
                    continue;
                };
                connected.store(true, Ordering::Relaxed);

                loop {
                    match rx.try_recv() {
                        Ok(data) => {
                            let len = (data.len() as u32).to_le_bytes();
                            if stream.write_all(&len).is_err() || stream.write_all(&data).is_err() {
                                break;
                            }
                        }
                        Err(TryRecvError::Empty) => {
                            std::thread::sleep(Duration::from_millis(1));
                        }
                        Err(TryRecvError::Disconnected) => return,
                    }
                }
            }
        })
        .expect("spawn lidar thread")
}
