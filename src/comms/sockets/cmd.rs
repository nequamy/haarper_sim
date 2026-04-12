use std::{io::Read, os::unix::net::UnixListener, thread::JoinHandle, time::Duration};

use crossbeam_channel::Sender;

use crate::comms::channels::CommandMessage;

pub fn spawn_cmd_thread(cmd_tx: Sender<CommandMessage>) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("haarper_cmd".into())
        .spawn(move || {
            let path = "/tmp/haarper_cmd.sock";
            let _ = std::fs::remove_file(path);
            let listener = UnixListener::bind(path).expect("bind cmd socket");

            loop {
                let Ok((mut stream, _)) = listener.accept() else {
                    continue;
                };
                stream
                    .set_read_timeout(Some(Duration::from_millis(100)))
                    .ok();

                let mut buf = [0u8; 20];
                loop {
                    match stream.read_exact(&mut buf) {
                        Ok(()) => {
                            let steering = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                            let duty = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
                            let _ = cmd_tx.try_send(CommandMessage { duty, steering });
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                        Err(_) => break,
                    }
                }
            }
        })
        .expect("spawn cmd thread")
}
