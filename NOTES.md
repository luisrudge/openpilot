02-02-2022
==========

Starting again from the beginning...

## Mount system as rw (read-write)
```
$ sudo su
$ mount -o rw,remount /
```

## Install useful tools
```
$ apt update
$ apt install -y nano vim wget htop tmux
```

## Update hostname (optional)
Change it to something you like! (keep it simple, no special chars/spaces)
```
$ vim /etc/hostname
```
```
frici
```

Update `/etc/hosts` to match.

```
$ vim /etc/hosts
```
```
127.0.0.1       localhost.localdomain   localhost
::1         ubuntu-phablet  localhost6.localdomain6 localhost6
127.0.1.1       ubuntu-phablet  ub-fajita   # <-- add your hostname here, matching exactly what you put in /etc/hostname

# ... the rest of the file ...
```


## Making space in the `system` partition

```
$ df -h
Filesystem                       Size  Used Avail Use% Mounted on
udev                             2.7G  776K  2.7G   1% /dev
tmpfs                            560M  1.5M  559M   1% /run
/dev/sda17                       108G  1.8G  106G   2% /userdata
/dev/disk/by-partlabel/system_a  2.7G  2.5G  180M  94% /
/dev/loop0                       533M  459M   63M  89% /android
...
```

There is not much space to work with to install packages in the system partition (94% used!). To fix this we can move the entire `/usr` directory to the `userdata` partition and create a bind mount so that it can still be accessed at `/usr`.

```
# Create the new directory
$ mkdir /userdata/usr

# Copy contents of /usr, preserving ownership and permissions
$ find . -depth -print0 | sudo cpio --null --sparse -pvd /userdata/usr/
```

Now we need to mount the new directory to `/usr` before we can delete the old files and make space.

```
$ mount -o bind,suid /userdata/usr /usr
$ touch /usr/TEST
$ ls /usr
# see TEST!
```

To do this permanently, we would normally modify `/etc/fstab`. However, Ubuntu Touch is weird and this file is generated on boot so we can't modify it. Instead, we can add an entry to `/lib/init/fstab`.

```
$ vim /lib/init/fstab
```
```
# TODO
```
