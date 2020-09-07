///
///	@file mediaplayer.cpp	@brief A software HD device plugin for VDR.
///
///	Copyright (c) 2020 by zille.  All Rights Reserved.
///
///	Contributor(s):
///
///	License: AGPLv3
///
///	This program is free software: you can redistribute it and/or modify
///	it under the terms of the GNU Affero General Public License as
///	published by the Free Software Foundation, either version 3 of the
///	License.
///
///	This program is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
///	GNU Affero General Public License for more details.
///
///	$Id$
//////////////////////////////////////////////////////////////////////////////


#include <string>
using std::string;
#include <fstream>
using std::ifstream;

#include <vdr/player.h>
#include <vdr/videodir.h>

#include "mediaplayer.h"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>

#include "softhddev.h"
#include "audio.h"
}

// fix temporary array error in c++1x
#ifdef av_err2str
#undef av_err2str
av_always_inline char* av_err2str(int errnum)
{
	static char str[AV_ERROR_MAX_STRING_SIZE];
	memset(str, 0, sizeof(str));
	return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}
#endif

//////////////////////////////////////////////////////////////////////////////
//	cPlayer Mediaplayer
//////////////////////////////////////////////////////////////////////////////

cSoftHdPlayer::cSoftHdPlayer(const char *File)
:cPlayer (pmAudioVideo)
{
//	pPlayer= this;
	Path = (char *) malloc(1 + strlen(File));
	strcpy(Path, File);
	Pause = 0;
//	fprintf(stderr, "cSoftHdPlayer: Player gestartet.\n");
}

cSoftHdPlayer::~cSoftHdPlayer()
{
	StopFile = 1;
//	fprintf(stderr, "cSoftHdPlayer: Player beendet.\n");
}

void cSoftHdPlayer::Activate(bool On)
{
//	fprintf(stderr, "cSoftHdPlayer: Activate %s\n", On ? "On" : "Off");
	if (On)
		Start();
}

void cSoftHdPlayer::Action(void)
{
//	fprintf(stderr, "cSoftHdPlayer: Action\n");
	Player(Path);

	sleep (1);
	cSoftHdControl::Control()->Close = 1;
}

void cSoftHdPlayer::Player(const char *url)
{
	AVPacket packet;
	AVCodec *video_codec;
//	AVCodec *audio_codec;
	int err = 0;
	int audio_stream_index = 0;
	int video_stream_index;
	int jump_stream_index = 0;
	int start_time;

	StopFile = 0;

	// get format from file
	AVFormatContext *format = avformat_alloc_context();
	if (avformat_open_input(&format, url, NULL, NULL) != 0) {
		fprintf(stderr, "Mediaplayer: Could not open file '%s'\n", url);
		return;
	}
#ifdef MEDIA_DEBUG
	av_dump_format(format, -1, url, 0);
#endif
	if (avformat_find_stream_info(format, NULL) < 0) {
		fprintf(stderr, "Mediaplayer: Could not retrieve stream info from file '%s'\n", url);
		return;
	}

	for (unsigned int i = 0; i < format->nb_streams; i++) {
		if (format->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
			SetAudioCodec(format->streams[i]->codecpar->codec_id,
				format->streams[i]->codecpar, &format->streams[i]->time_base);
			audio_stream_index = jump_stream_index = i;
			break;
		}
	}

	video_stream_index = av_find_best_stream(format, AVMEDIA_TYPE_VIDEO,
		-1, -1, &video_codec, 0);

	if (video_stream_index < 0) {
		fprintf(stderr, "Player: stream does not seem to contain video\n");
	} else {
		 SetVideoCodec(video_codec->id,
			format->streams[video_stream_index]->codecpar,
			&format->streams[video_stream_index]->time_base);
		jump_stream_index = video_stream_index;
	}

	duration = format->duration / AV_TIME_BASE;
	start_time = format->start_time / AV_TIME_BASE;

	while (!StopFile) {
		err = av_read_frame(format, &packet);
		if (err == 0) {
repeat:
			if (audio_stream_index == packet.stream_index) {
				if (!PlayAudioPkts(&packet)) {
					usleep(packet.duration * AV_TIME_BASE *
						format->streams[audio_stream_index]->time_base.num	// ???
						/ format->streams[audio_stream_index]->time_base.den);
					goto repeat;
				}
				current_time = AudioGetClock() / 1000 - start_time;
			}

			if (video_stream_index == packet.stream_index) {
				if (!PlayVideoPkts(&packet)) {
					usleep(packet.duration * AV_TIME_BASE *
						format->streams[video_stream_index]->time_base.num	// ???
						/ format->streams[video_stream_index]->time_base.den);
					goto repeat;
				}
			}
		} else {
			fprintf(stderr, "Player: av_read_frame error: %s\n",
				av_err2str(err));
			StopFile = 1;
			continue;
		}

		while (Pause) {
			sleep(1);
		}

		if (Jump) {
			av_seek_frame(format, format->streams[jump_stream_index]->index,
				packet.pts + (int64_t)(Jump *		// - BufferOffset
				format->streams[jump_stream_index]->time_base.den /
				format->streams[jump_stream_index]->time_base.num), 0);
			Clear();
			Jump = 0;
		}

		if (StopFile)
			Clear();

		av_packet_unref(&packet);
	}

	duration = 0;
	current_time = 0;

	avformat_close_input(&format);
	avformat_free_context(format);
}

//////////////////////////////////////////////////////////////////////////////
//	cControl Mediaplayer
//////////////////////////////////////////////////////////////////////////////

cSoftHdControl *cSoftHdControl::pControl = NULL;

/**
**	Player control constructor.
*/
cSoftHdControl::cSoftHdControl(const char *File)
:cControl(pPlayer = new cSoftHdPlayer(File))
{
//	fprintf(stderr, "cSoftHdControl: Player gestartet.\n");
	pControl = this;
	Close = 0;
	pOsd = NULL;
}

/**
**	Player control destructor.
*/
cSoftHdControl::~cSoftHdControl()
{
	delete pPlayer;
	pPlayer = NULL;
#ifdef MEDIA_DEBUG
	fprintf(stderr, "cSoftHdControl: Player beendet.\n");
#endif
}

void cSoftHdControl::Hide(void)
{
#ifdef MEDIA_DEBUG
	fprintf(stderr, "[cSoftHdControl] Hide\n");
#endif
	if (pOsd) {
		delete pOsd;
		pOsd = NULL;
	}
}

void cSoftHdControl::ShowProgress(void)
{
	if (!pOsd) {
#ifdef MEDIA_DEBUG
		fprintf(stderr, "[cSoftHdControl] ShowProgress get OSD\n");
#endif
		pOsd = Skins.Current()->DisplayReplay(false);
	}

	pOsd->SetTitle(pPlayer->GetTitle());
	pOsd->SetProgress(pPlayer->CurrentTime(), pPlayer->TotalTime());
	pOsd->SetCurrent(IndexToHMSF(pPlayer->CurrentTime(), false, 1));
	pOsd->SetTotal(IndexToHMSF(pPlayer->TotalTime(), false, 1));

	Skins.Flush();
}

/**
**	Handle a key event.
**
**	@param key	key pressed
*/
eOSState cSoftHdControl::ProcessKey(eKeys key)
{
	switch (key) {
		case kNone:
			if (pOsd)
				ShowProgress();
			if (Close) {
				Hide();
				return osStopReplay;
			}
			break;

		case kOk:
			if (pOsd) {
				Hide();
			} else {
				ShowProgress();
			}
			break;

		case kPlay:
			if (pPlayer->Pause) {
				pPlayer->Pause = 0;
				Play();
			}
			break;

		case kGreen:
			pPlayer->Jump = -60;
		break;

		case kYellow:
			pPlayer->Jump = 60;
		break;

		case kBlue:
			Hide();
			pPlayer->StopFile = 1;
			return osStopReplay;

		case kPause:
			if (pPlayer->Pause) {
				pPlayer->Pause = 0;
				Play();
			} else {
				pPlayer->Pause = 1;
				Freeze();
			}
			break;

		default:
			break;
	}

	return osContinue;
}


//////////////////////////////////////////////////////////////////////////////
//	cOsdMenu
//////////////////////////////////////////////////////////////////////////////
cSoftHdControl *cSoftHdMenu::Control = NULL;

/**
**	Soft device menu constructor.
*/
cSoftHdMenu::cSoftHdMenu(const char *title, int c0, int c1, int c2, int c3,
    int c4)
:cOsdMenu(title, c0, c1, c2, c3, c4)
{
	MainMenu();
}

/**
**	Soft device menu destructor.
*/
cSoftHdMenu::~cSoftHdMenu()
{
}

/**
**	Create main menu.
*/
void cSoftHdMenu::MainMenu(void)
{
	int current;
	int missed;
	int duped;
	int dropped;
	int counter;

	current = Current();		// get current menu item index
	Clear();				// clear the menu

	Add(new cOsdItem(hk(tr(" Play file")), osUser1));
	Add(new cOsdItem(NULL, osUnknown, false));
	Add(new cOsdItem(NULL, osUnknown, false));
	GetStats(&missed, &duped, &dropped, &counter);
	Add(new cOsdItem(cString::sprintf(tr
		(" Frames missed(%d) duped(%d) dropped(%d) total(%d)"), missed,
		duped, dropped, counter), osUnknown, false));

	SetCurrent(Get(current));		// restore selected menu entry
	Display();				// display build menu
}

/**
**	Create sub menu find file or make a PL.
**
**	@param SearchPath	path to start search mediafile
**	@param playlist		if there is a PL write PL else make a new menu
*/
void cSoftHdMenu::FindFile(string SearchPath, FILE *playlist)
{
	struct dirent **DirList;
	int n, i;
	const char * sp;

	if (!SearchPath.size())
		sp = "/";
	else sp = SearchPath.c_str();

	if (!playlist) {
		Clear();
		if (SearchPath.size())
			Add(new cOsdItem(tr("[..]")));
	}

	if ((n = scandir(sp, &DirList, NULL, alphasort)) == -1) {
		esyslog("Mediaplayer: scanning directory %s failed (%d): %m\n", sp, errno);
	} else {
		for (i = 0; i < n; i++) {
			if (DirList[i]->d_type == DT_DIR && DirList[i]->d_name[0] != '.') {
				Add(new cOsdItem(tr(DirList[i]->d_name)));
			}
		}
		for (i = 0; i < n; i++) {
			if (DirList[i]->d_type == DT_REG && DirList[i]->d_name[0] != '.') {
				Add(new cOsdItem(tr(DirList[i]->d_name)));
			}
		}
	}

	if (!playlist) {
		SetHelp( "Play", NULL, NULL, NULL);
//		SetHelp(Control->Player->Running ? NULL : "Set new PL",
//			Control->Player->Running ? "Play Menu" : "Select PL");
		Display();
	}
//	MenuOpen = 2;
}

/**
**	Handle key event.
**
**	@param key	key event
*/
eOSState cSoftHdMenu::ProcessKey(eKeys key)
{
	eOSState state;
	cOsdItem *item;

	item = (cOsdItem *) Get(Current());
	state = cOsdMenu::ProcessKey(key);

	switch (key) {
		case kOk:
			if (strcasestr(item->Text(), "[..]")) {
				string NewPathString = Path.substr(0,Path.find_last_of("/"));
				Path = NewPathString;
				FindFile(Path.c_str(), NULL);
			} else {
				if (strcasestr(item->Text(), ".MP")) {
					fprintf(stderr, "[ProcessKey] %s:\n", item->Text());
					string NewPathString = Path + "/" + item->Text();
					cControl::Launch(Control = new cSoftHdControl(NewPathString.c_str()));
					Clear();	// clear the menu
					return osEnd;
				} else { // Fixme! Control this is a folder!!!
					string NewPathString = Path + "/" + item->Text();
					Path = NewPathString;
					FindFile(NewPathString.c_str(), NULL);
				}
			}
			break;
		case kRed:
			if (item->Text()) {
				string NewPathString = Path + "/" + item->Text();
				cControl::Launch(Control = new cSoftHdControl(NewPathString.c_str()));
				Clear();	// clear the menu
				return osEnd;
			}
			break;
		default:
			break;
	}

	switch (state) {
		case osUser1:			// Play File
			Path = cVideoDirectory::Name();
			FindFile(Path, NULL);
		default:
//			MainMenu();
			break;
	}
	return state;
}
