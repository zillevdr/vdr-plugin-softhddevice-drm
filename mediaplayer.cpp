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
}



//////////////////////////////////////////////////////////////////////////////
//	cPlayer Mediaplayer
//////////////////////////////////////////////////////////////////////////////

cSoftHdPlayer::cSoftHdPlayer(const char *File)
:cPlayer (pmAudioVideo)
{
//	pPlayer= this;
	Path = (char *) malloc(1 + strlen(File));
	strcpy(Path, File);
	fprintf(stderr, "cSoftHdPlayer: Player gestartet.\n");
}

cSoftHdPlayer::~cSoftHdPlayer()
{
	StopFile = 1;
	fprintf(stderr, "cSoftHdPlayer: Player beendet.\n");
}

void cSoftHdPlayer::Activate(bool On)
{
	fprintf(stderr, "cSoftHdPlayer: Activate %s\n", On ? "On" : "Off");
	if (On)
		Start();
}

void cSoftHdPlayer::Action(void)
{
	fprintf(stderr, "cSoftHdPlayer: Action\n");
	PlayFile(Path);

	sleep (10);
	cSoftHdControl::Control()->Close = 1;
}

void cSoftHdPlayer::PlayFile(const char *path)
{
	AVPacket packet;
	AVCodec *video_codec;
//	AVCodec *audio_codec;
	int err = 0;
	int audio_stream_index = 0;
	int video_stream_index;

	StopFile = 0;

	// get format from file
	AVFormatContext* format = avformat_alloc_context();
	if (avformat_open_input(&format, path, NULL, NULL) != 0) {
//		esyslog("Mediaplayer: Could not open file '%s'\n", path);
		fprintf(stderr, "Mediaplayer: Could not open file '%s'\n", path);
		return;
	}

	av_dump_format(format, -1, path, 0);

	if (avformat_find_stream_info(format, NULL) < 0) {
//		esyslog("Mediaplayer: Could not retrieve stream info from file '%s'\n", path);
		fprintf(stderr, "Mediaplayer: Could not retrieve stream info from file '%s'\n", path);
		return;
	}

	int secs  = format->duration / AV_TIME_BASE;
	int us    = format->duration % AV_TIME_BASE;
	int mins  = secs / 60;
	secs %= 60;
	int hours = mins / 60;
	mins %= 60;
	fprintf(stderr, "Player: %s codec: %s duration %02d:%02d:%02d.%02d\n",
		format->url, format->iformat->name, hours, mins, secs,
						(100 * us) / AV_TIME_BASE);

	fprintf(stderr, "Player:  nb_streams %d  %s\n", format->nb_streams,
		av_get_media_type_string(format->streams[0]->codecpar->codec_type));

	for (unsigned int i = 0; i < format->nb_streams; i++) {

		fprintf(stderr, "Player: i %d nb_streams %d  %s %s\n", i, format->nb_streams,
			av_get_media_type_string(format->streams[i]->codecpar->codec_type),
			avcodec_get_name(format->streams[i]->codecpar->codec_id));

		if (format->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
			SetAudioCodec(format->streams[i]->codecpar->codec_id,
				format->streams[i]->codecpar);
			audio_stream_index = i;
			break;
		}
	}

	video_stream_index = av_find_best_stream(format, AVMEDIA_TYPE_VIDEO,
		-1, -1, &video_codec, 0);
	if (video_stream_index < 0)
		fprintf(stderr, "Player: stream does not seem to contain video\n");
	else  SetVideoCodec(video_codec->id,
				format->streams[video_stream_index]->codecpar);

	while (err == 0 && !StopFile) {
		err = av_read_frame(format, &packet);
		if (err == 0) {
repeat:
			if (audio_stream_index == packet.stream_index) {
				if (!PlayAudioPkts(&packet)) {
					usleep(packet.duration * AV_TIME_BASE *
						format->streams[0]->time_base.num 
						/ format->streams[0]->time_base.den);
					goto repeat;
				}
			}

			if (video_stream_index == packet.stream_index) {
				if (!PlayVideoPkts(&packet)) {
					usleep(packet.duration * AV_TIME_BASE *
						format->streams[0]->time_base.num 
						/ format->streams[0]->time_base.den);
					goto repeat;
				}
			}
		}

		if (StopFile)
			DeviceClear();

		av_packet_unref(&packet);
	}

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
	fprintf(stderr, "cSoftHdControl: Player gestartet.\n");
	pControl = this;
	Close = 0;
}

/**
**	Player control destructor.
*/
cSoftHdControl::~cSoftHdControl()
{
	delete pPlayer;
	pPlayer = NULL;

	fprintf(stderr, "cSoftHdControl: Player beendet.\n");
}

void cSoftHdControl::Hide(void)
{
	fprintf(stderr, "[cSoftHdControl] Hide\n");
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
			if (Close)
				return osStopReplay;
			break;

		case kBlue:
			return osStopReplay;

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
			} else { // Fixme! Control this is a folder!!!
				string NewPathString = Path + "/" + item->Text();
				Path = NewPathString;
				FindFile(NewPathString.c_str(), NULL);
			}
			break;
		case kRed:
			if (item->Text()) {	// make compiler happy
				string NewPathString = Path + "/" + item->Text();
				fprintf(stderr, "ProcessKey: Play file path %s\n", NewPathString.c_str());
				cControl::Launch(Control = new cSoftHdControl(NewPathString.c_str()));
				fprintf(stderr, "ProcessKey: Clear()\n");
				Clear();	// clear the menu
				fprintf(stderr, "ProcessKey: return osEnd\n");
				return osEnd;
			}
			break;
		default:
			break;
	}

	switch (state) {
		case osUser1:			// Play File
			Path = cVideoDirectory::Name();
			fprintf(stderr, "ProcessKey: Play file path %s\n", Path.c_str());
			FindFile(Path, NULL);
		default:
//			MainMenu();
			break;
	}
	return state;
}
