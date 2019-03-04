#!/bin/bash
# FFMPEG script
echo "Installing libraries for FFMPEG!"
sudo apt-get -y install autoconf automake build-essential git libass-dev libfreetype6-dev \
  libsdl2-dev libtheora-dev libtool libva-dev libvdpau-dev libvorbis-dev libxcb1-dev libxcb-shm0-dev \
  libxcb-xfixes0-dev pkg-config texinfo wget zlib1g-dev
mkdir ~/ffmpeg_sources ~/bin

cd ~/ffmpeg_sources
echo "Installing nasm lib"
wget http://www.nasm.us/pub/nasm/releasebuilds/2.13.01/nasm-2.13.01.tar.bz2
tar xjvf nasm-2.13.01.tar.bz2
rm nasm-2.13.01.tar.bz2
cd nasm-2.13.01
./autogen.sh
PATH="$HOME/bin:$PATH" ./configure --prefix="$HOME/ffmpeg_build" --bindir="$HOME/bin"
make
make install

echo "Installing yasm lib"
sudo apt-get install yasm

echo "Installing x264 lib"
cd ~/ffmpeg_sources
git clone --depth 1 http://git.videolan.org/git/x264
cd x264
PATH="$HOME/bin:$PATH" PKG_CONFIG_PATH="$HOME/ffmpeg_build/lib/pkgconfig" ./configure --prefix="$HOME/ffmpeg_build" --bindir="$HOME/bin" --enable-static --enable-shared
PATH="$HOME/bin:$PATH" make
make install

echo "Installing x265"
sudo apt-get install -y cmake mercurial
cd ~/ffmpeg_sources
hg clone https://bitbucket.org/multicoreware/x265
cd ~/ffmpeg_sources/x265/build/linux
PATH="$HOME/bin:$PATH" cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX="$HOME/ffmpeg_build" -DENABLE_SHARED:bool=off ../../source
PATH="$HOME/bin:$PATH" make
make install

echo "Installing remaining dependencies"
sudo apt-get -y install libx265-dev libfdk-aac-dev libmp3lame-dev libopus-dev libvpx-dev

echo "Installing FFMPEG"
cd ~/ffmpeg_sources
wget http://ffmpeg.org/releases/ffmpeg-2.8.11.tar.bz2
tar xjvf ffmpeg-2.8.11.tar.bz2
rm ffmpeg-2.8.11.tar.bz2
cd ffmpeg-2.8.11
PATH="$HOME/bin:$PATH" PKG_CONFIG_PATH="$HOME/ffmpeg_build/lib/pkgconfig" ./configure \
  --prefix="$HOME/ffmpeg_build" \
  --pkg-config-flags="--static" \
  --extra-cflags="-I$HOME/ffmpeg_build/include" \
  --extra-cflags="-fPIC" \
  --extra-ldflags="-L$HOME/ffmpeg_build/lib" \
  --extra-libs="-lpthread -lm" \
  --bindir="$HOME/bin" \
  --enable-gpl \
  --enable-libass \
  --enable-libfdk-aac \
  --enable-libfreetype \
  --enable-libmp3lame \
  --enable-libopus \
  --enable-libtheora \
  --enable-libvorbis \
  --enable-libvpx \
  --enable-libx264 \
  --enable-libx265 \
  --enable-avresample \
  --enable-nonfree \
  --enable-pic \
  --enable-shared
PATH="$HOME/bin:$PATH" make
make install
hash -r
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$HOME/eb/ffmpeg_build/lib/pkgconfig
export PKG_CONFIG_LIBDIR=$PKG_CONFIG_LIBDIR:$HOME/eb/ffmpeg_build/lib/
export LD_LIBRARY_PATH=$HOME/eb/ffmpeg_build/lib/
sudo sh -c "echo '$HOME/ffmpeg_build/lib' >> /etc/ld.so.conf"
sudo ldconfig
