# Copyright 1999-2015 Gentoo Foundation
# Distributed under the terms of the GNU General Public License v2
# $Id$

EAPI=5

ETYPE=sources
K_DEFCONFIG="rockchip_defconfig"
K_SECURITY_UNSUPPORTED=1
EXTRAVERSION="-${PN}/-*"
inherit kernel-2
detect_version
detect_arch

inherit git-2 versionator
EGIT_REPO_URI=https://github.com/rockchip-linux/kernel.git
EGIT_PROJECT="rockchip-linux.git"
EGIT_BRANCH="release-$(get_version_component_range 1-2)"

DESCRIPTION="Rockchip kernel sources"
HOMEPAGE="https://github.com/rockchip-linux"

KEYWORDS=""

src_unpack() {
	git-2_src_unpack
	unpack_set_extraversion
}
