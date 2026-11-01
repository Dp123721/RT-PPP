! Persistent line-oriented adapter for the original GPT3/VMF3 routines.
! The working directory must contain gpt3_5.grd.
program gpt3_vmf3_worker
    implicit none

    character(len=2048) :: line
    character(len=16) :: command
    integer :: io_status
    double precision :: mjd, lat, lon, h_ell, zd, mfh, mfw
    double precision, dimension(1) :: lat_v, lon_v, h_v
    double precision, dimension(1) :: p, t, dt, tm, e, ah, aw, la, undu
    double precision, dimension(1) :: gn_h, ge_h, gn_w, ge_w

    do
        read(*, '(A)', iostat=io_status) line
        if (io_status /= 0) exit
        command = ''
        read(line, *, iostat=io_status) command
        if (io_status /= 0) then
            write(*, '(A)') 'ERR malformed_request'
            flush(6)
            cycle
        end if

        select case (trim(adjustl(command)))
        case ('PING')
            write(*, '(A)') 'OK PONG'
        case ('QUIT')
            write(*, '(A)') 'OK BYE'
            flush(6)
            exit
        case ('MAP')
            read(line, *, iostat=io_status) command, mjd, lat, lon, h_ell, zd
            if (io_status /= 0) then
                write(*, '(A)') 'ERR malformed_map_request'
            else
                lat_v(1) = lat
                lon_v(1) = lon
                h_v(1) = h_ell
                call gpt3_5(mjd, lat_v, lon_v, h_v, 1, 0, p, t, dt, tm, e, ah, aw, la, undu, gn_h, ge_h, gn_w, ge_w)
                call vmf3_ht(ah(1), aw(1), mjd, lat, lon, h_ell, zd, mfh, mfw)
                if (mfh /= mfh .or. mfw /= mfw .or. tm(1) /= tm(1)) then
                    write(*, '(A)') 'ERR non_finite_result'
                else
                    write(*, '(A,1X,ES24.16E3,1X,ES24.16E3,1X,ES24.16E3)') 'OK', mfh, mfw, tm(1)
                end if
            end if
        case default
            write(*, '(A)') 'ERR unknown_command'
        end select
        flush(6)
    end do
end program gpt3_vmf3_worker
