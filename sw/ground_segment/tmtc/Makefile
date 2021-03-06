# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2003-2006 Pascal Brisset, Antoine Drouin
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#

# Quiet
Q=@

include ../../../conf/Makefile.local
include ../../Makefile.ocaml
CONF = ../../../conf
VAR = ../../../var

INCLUDES= -I ../multimon
PKG = -package glibivy,pprz
LINKPKG = $(PKG) -linkpkg -dllpath-pkg pprz
XPKG = -package pprz.xlib
XLINKPKG = $(XPKG) -linkpkg -dllpath-pkg pprz.xlib

LIBMULTIMONCMA=../multimon/multimon.cma
LIBMULTIMONDLL= multimon.cma -dllpath $(PAPARAZZI_SRC)/sw/ground_segment/multimon

SERVERCMO = server_globals.cmo aircraft.cmo wind.cmo airprox.cmo kml.cmo fw_server.ml rotorcraft_server.ml server.cmo
SERVERCMX = $(SERVERCMO:.cmo=.cmx)


all: link server messages settings dia diadec ivy_tcp_aircraft ivy_tcp_controller broadcaster ivy2udp ivy_serial_bridge app_server ivy2nmea

opt: server.opt

clean:
	$(Q)rm -f link server messages settings dia diadec *.bak *~ core *.o .depend *.opt *.out *.cm* ivy_tcp_aircraft ivy_tcp_controller broadcaster ivy2udp ivy_serial_bridge app_server gpsd2ivy c_ivy_client_example_1 c_ivy_client_example_2 c_ivy_client_example_3 ivy2nmea

messages : messages.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) gtkInit.cmo $<

settings : settings.cmo ../cockpit/page_settings.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(XLINKPKG) gtkInit.cmo -I ../cockpit gtk_save_settings.cmo saveSettings.cmo page_settings.cmo $<

server : $(SERVERCMO) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(SERVERCMO)

server.opt :  $(SERVERCMX) $(LIBPPRZCMXA)
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(INCLUDES) -o $@ -package glibivy,pprz -linkpkg $(SERVERCMX)

link : link.cmo $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBMULTIMONDLL) $<


ivy_tcp_aircraft : ivy_tcp_aircraft.cmo  $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBMULTIMONDLL) $<


ivy_tcp_controller : ivy_tcp_controller.cmo  $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBMULTIMONDLL) $<


broadcaster : broadcaster.cmo $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBMULTIMONDLL) $<


ivy2udp : ivy2udp.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $<


dia : dia.cmo $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBMULTIMONDLL) $<


diadec : diadec.cmo $(LIBMULTIMONCMA) $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) $(LIBMULTIMONDLL) $<


150m : 150m.cmo $(LIBPPRZCMA)
	@echo OL $@
	$(Q)$(OCAMLC) $(INCLUDES) -o $@ $(LINKPKG) gtkInit.cmo $<

settings.cmo : INCLUDES += -I ../cockpit
settings.cmo : ../cockpit/page_settings.cmi

%.cmo : %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $(PKG) -c $<

%.cmx : %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(INCLUDES) $(PKG) -c $<

%.cmi : %.mli
	@echo OC $<
	$(Q)$(OCAMLC) $(INCLUDES) $(PKG) $<


CC = gcc
GLIB_CFLAGS  = -Wall  `pkg-config glib-2.0 --cflags` -fPIC
GLIB_LDFLAGS =  `pkg-config glib-2.0 --libs` -lglibivy -lpcre -fPIC
GTK_CFLAGS  = -Wall  `pkg-config gtk+-2.0 --cflags` -fPIC
GTK_LDFLAGS =  `pkg-config gtk+-2.0 --libs` -lglibivy -lpcre -fPIC

GTK_CFLAGS  = $(shell pkg-config gtk+-2.0 --cflags)
GTK_LDFLAGS =  $(shell pkg-config gtk+-2.0 --libs) $(shell pcre-config --libs)

GLIBIVY_CFLAGS = -Wall -fPIC $(shell pkg-config --cflags ivy-glib)
GLIBIVY_LDFLAGS = $(shell pkg-config --libs ivy-glib) $(shell pcre-config --libs)

#
# try to find include locations on OSX
#
UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
  IVY_C_LIB_INC = $(shell if test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; elif test -d /opt/local/lib; then echo "-L/opt/local/lib"; fi)
  IVY_C_INCLUDES = $(shell if test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; elif test -d /opt/local/include; then echo "-I/opt/local/include"; fi)
  GLIBIVY_CFLAGS += $(IVY_C_LIB_INC) $(IVY_C_INCLUDES)
endif


app_server: app_server.c
	@echo OL $@
	$(Q)$(CC) $(shell pkg-config libxml-2.0 gio-2.0 ivy-glib --cflags) -o $@ $^ -lm -lz $(shell pkg-config libxml-2.0 gio-2.0 ivy-glib libpcre --libs)

gpsd2ivy: gpsd2ivy.c
	$(CC) $(GLIBIVY_CFLAGS) -o $@ $< $(GLIBIVY_LDFLAGS) -lgps

ivy2nmea: ivy2nmea.c
	@echo OL $@
	$(Q)$(CC) $(GLIBIVY_CFLAGS) -o $@ $< $(GLIBIVY_LDFLAGS)

c_ivy_client_example_1: c_ivy_client_example_1.c
	$(CC) $(GLIBIVY_CFLAGS) -o $@ $< $(GLIBIVY_LDFLAGS)

c_ivy_client_example_2: c_ivy_client_example_2.c
	$(CC) $(GLIBIVY_CFLAGS) -o $@ $< $(GLIBIVY_LDFLAGS)

c_ivy_client_example_3: c_ivy_client_example_3.c
	$(CC) $(GLIBIVY_CFLAGS) $(GTK_CFLAGS) -o $@ $< $(GLIBIVY_LDFLAGS) $(GTK_LDFLAGS)

ivy_serial_bridge: ivy_serial_bridge.c
	@echo OL $@
	$(Q)$(CC) $(GLIBIVY_CFLAGS) $(GTK_CFLAGS) -o $@ $< $(GLIBIVY_LDFLAGS) $(GTK_LDFLAGS)


.PHONY: all opt clean

#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) -I ../multimon *.ml* > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
