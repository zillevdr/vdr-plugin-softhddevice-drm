# Copyright 1999-2014 Gentoo Foundation
# Distributed under the terms of the GNU General Public License v2
# $Header: $

EAPI="5"

inherit vdr-plugin-2 git-2

RESTRICT="test"

EGIT_REPO_URI="git://github.com/zillevdr/vdr-plugin-softhddevice.git"
KEYWORDS=""

DESCRIPTION="VDR Plugin: Software and GPU emulated HD output device"
HOMEPAGE="http://projects.vdr-developer.org/projects/show/plg-softhddevice"

LICENSE="AGPL-3"
SLOT="0"
IUSE="alsa debug +mmal oss"

RDEPEND=">=media-video/vdr-2
	alsa? ( media-libs/alsa-lib )
	mmal? ( =media-video/ffmpeg-9999 )"
#	mmal? ( sys-boot/raspberrypi-firmware )"

#VDR_CONFD_FILE="${FILESDIR}/confd-0.6.0"
#VDR_RCADDON_FILE="${FILESDIR}/rc-addon-0.6.0.sh"

pkg_setup() {
	vdr-plugin-2_pkg_setup

	append-cppflags -DHAVE_PTHREAD_NAME

	use debug && append-cppflags -DDEBUG -DOSD_DEBUG
}
src_prepare() {
	vdr-plugin-2_src_prepare

	BUILD_PARAMS+=" ALSA=$(usex alsa 1 0)"
	BUILD_PARAMS+=" OSS=$(usex oss 1 0)"
	BUILD_PARAMS+=" MMAL=$(usex mmal 1 0)"
}

src_install() {
	vdr-plugin-2_src_install

	nonfatal dodoc ChangeLog Todo
}
