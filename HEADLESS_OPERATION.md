# Headless Realtime Operation

The `--headless` mode starts the realtime streams stored in an INI file
without displaying the Qt Widgets interface. It shares the normal runtime:
network clients, RTCM decoding, PPP, GFS initialization, and output workers
remain active. The mode does not require X11 on Linux.

On Linux, the Qt 5 `offscreen` platform plugin must be installed alongside
the Qt runtime. Distribution package names vary; for a private Qt deployment,
include the matching `platforms/libqoffscreen.so` plugin and its shared-library
dependencies. Windows headless mode does not force this Unix platform plugin.

The first version intentionally supports realtime streams only. A saved
configuration with `ppp/processingMode=post` is rejected because post-process
task completion and exit status require a separate non-interactive contract.

## Linux

Start from the project root, or another deliberate service working directory,
because relative output paths in the INI are interpreted relative to the
process working directory:

```sh
cd /opt/rtppp
./build/linux/bin/RT-PPP --headless --config /etc/rtppp/rtppp_settings.ini
```

Use `Ctrl+C` for an interactive process, or send `SIGTERM` from a service
manager. Both request a normal Qt event-loop exit, which invokes the existing
shutdown path and finalizes output such as PPP JSON.

After building on Linux, verify startup without either common display server:

```sh
bash ./tests/headless_smoke.sh ./build/linux/bin/RT-PPP
```

The probe uses a local TCP test endpoint and exits after two seconds. It checks
the no-display startup path, not successful reception from a production caster.

For a short deployment probe:

```sh
./build/linux/bin/RT-PPP --headless \
  --config /etc/rtppp/rtppp_settings.ini \
  --run-seconds 20
```

The repository includes `deploy/rtppp.service`. Review its `User`, paths, and
permissions before installing it.

Install after reviewing paths and the configuration file permissions:

```sh
sudo install -m 0644 rtppp.service /etc/systemd/system/rtppp.service
sudo systemctl daemon-reload
sudo systemctl enable --now rtppp
sudo journalctl -u rtppp -f
```

Keep stream credentials in a configuration file readable only by the service
account. The existing INI format stores NTRIP credentials in plaintext.

## Windows

The same executable arguments work from PowerShell or an elevated service
wrapper:

```powershell
Set-Location D:\RT-PPP
.\build\vs\release\RT-PPP.exe --headless --config D:\RT-PPP\rtppp_settings.ini
```

Console close, Ctrl+C, and Ctrl+Break request the same graceful shutdown path.

## Command-line options

```text
--headless             Start saved realtime streams without showing a window
--config <file>        Read this INI instead of rtppp_settings.ini
--run-seconds <n>      Exit cleanly after n seconds; useful for service probes
--help                 Show all available options
```
