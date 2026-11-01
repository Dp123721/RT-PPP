# GPT3 + VMF3 worker

RT-PPP calls the original Fortran GPT3 and VMF3 routines through the
line-oriented `gpt3_vmf3_worker` process. The Fortran source files are not
translated to C++ and the application does not link against a Fortran
library.

The worker expects `gpt3_5.grd` in its own directory. It accepts:

```text
PING
MAP <mjd> <latitude_rad> <longitude_rad> <ellipsoidal_height_m> <zenith_distance_rad>
QUIT
```

The response to `MAP` is:

```text
OK <mfh> <mfw> <Tm_K>
```

Build and deploy on Windows from a shell with `gfortran` on `PATH`:

```powershell
./gpt3/build_worker.ps1 -OutputDirectory build/vs/debug/gpt3
```

On Linux:

```sh
./gpt3/build_worker.sh build/linux/gpt3
```

The scripts use static GCC/Fortran runtime linking where supported. If the
local toolchain does not provide static runtime libraries, remove the two
`-static-*` flags and deploy the matching runtime libraries with the worker.

The executable searches for the worker beside the application in `gpt3/`,
then in the repository-relative development paths. `RTPPP_GPT3_WORKER` can
override this location.

When `GPT3 + VMF3` is selected in PPP Settings, a worker or grid failure is a
hard PPP model error. There is no automatic fallback to `1/sin(E)`. The
`Default (1/sin E)` option remains independent and requires no worker.
