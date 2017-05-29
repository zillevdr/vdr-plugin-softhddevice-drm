# Copyright 1999-2017 Gentoo Foundation
# Distributed under the terms of the GNU General Public License v2
# $Id$

EAPI=5
inherit cmake-utils flag-o-matic git-r3

DESCRIPTION="Rockchip mpp libraries"
HOMEPAGE="https://github.com/rockchip-linux/mpp"
SRC_URI=""

#LICENSE="BSD"
SLOT="0"
#KEYWORDS=""
#IUSE=""

#DEPEND=""
#RDEPEND=""

EGIT_REPO_URI="https://github.com/rockchip-linux/mpp"
#EGIT_REPO_URI="https://github.com/LongChair/mpp"
EGIT_BRANCH="for_linux"

PATCHES=( "${FILESDIR}"/lfs.patch )

#pkg_setup() {
#	append-ldflags $(no-as-needed)
#}

src_configure() {
	local mycmakeargs=(
#		-DVMCS_INSTALL_PREFIX="/usr"
		-DCMAKE_INSTALL_PREFIX="/usr"
		-DCMAKE_BUILD_TYPE="DEBUG"
		-DHAVE_DRM="ON"
		-DRKPLATFORM="ON"
	)

	cmake-utils_src_configure
}

src_install() {
#	cmake-utils_src_install
	mkdir "${D}"usr/
	mkdir "${D}"usr/lib/
	cp "${S}"_build/mpp/librockchip_mpp.* "${D}"usr/lib/
	cp "${S}"_build/mpp/legacy/librockchip_vpu.* "${D}"usr/lib/

	mkdir "${D}"usr/include/
	mkdir "${D}"usr/include/rockchip/
	cp "${S}"/inc/* "${D}"usr/include/rockchip/



#	cp "${S}"/inc/mpp_frame.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/mpp_packet.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/rk_type.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/mpp_err.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/mpp_task.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/rk_mpi_cmd.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/vpu_api.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/vpu.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/mpp_buffer.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/rk_mpi.h "${D}"usr/include/rockchip/
#	cp "${S}"/inc/mpp_test_meta.h "${D}"usr/include/rockchip/

#	insinto /lib/udev/rules.d
#	doins "${FILESDIR}"/92-local-vchiq-permissions.rules

#	dodir /usr/share/doc/${PF}
#	mv "${D}"/usr/src/hello_pi "${D}"/usr/share/doc/${PF}/
#	rmdir "${D}"/usr/src
}
