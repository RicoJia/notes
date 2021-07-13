find_library( GTK3_LIBRARY
		NAMES gtk-3)
find_path(GTK3_INCLUDE_DIR
		NAMES gtk/gtk.h
		PATH_SUFFIXES gtk-3.0)
find_path(GLIB_INCLUDE_DIR
		NAMES glib.h
		PATH_SUFFIXES glib-2.0)
find_path(GLIBCONFIG_INCLUDE_DIR
		NAMES glibconfig.h
		HINTS /usr/lib/x86_64-linux-gnu
		PATH_SUFFIXES glib-2.0/include) 
find_path(PANGO_INCLUDE_DIR
		NAMES pango/pango.h
		PATH_SUFFIXES pango-1.0)
find_path(CAIRO_INCLUDE_DIR
		NAMES cairo.h
		PATH_SUFFIXES cairo)
find_path(GDK_PIXBUF_INCLUDE_DIR
		NAMES gdk-pixbuf/gdk-pixbuf.h
		PATH_SUFFIXES gdk-pixbuf-2.0)
find_path(ATK_INCLUDE_DIR
		NAMES atk/atk.h
		PATH_SUFFIXES atk-1.0)
find_library(GIO_LIBRARY
		NAMES gio-2.0)
find_library(GOBJECT_LIBRARY
		NAMES gobject-2.0)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTK3 DEFAULT_MSG
	GTK3_LIBRARY 
	GIO_LIBRARY 
	GOBJECT_LIBRARY
	GTK3_INCLUDE_DIR 
	GLIB_INCLUDE_DIR 
	GLIBCONFIG_INCLUDE_DIR
	PANGO_INCLUDE_DIR 
	CAIRO_INCLUDE_DIR 
	GDK_PIXBUF_INCLUDE_DIR 
	ATK_INCLUDE_DIR)

if(GTK3_FOUND)

    set(GTK3_INCLUDE_DIRS  
		${GTK3_INCLUDE_DIR}  
		${GLIB_INCLUDE_DIR} 
		${GLIBCONFIG_INCLUDE_DIR}
		${PANGO_INCLUDE_DIR}  
		${CAIRO_INCLUDE_DIR} 
		${GDK_PIXBUF_INCLUDE_DIR} 
		${ATK_INCLUDE_DIR})
		
    set(GTK3_LIBRARIES  
		${GTK3_LIBRARY} 
		${GIO_LIBRARY} 
		${GOBJECT_LIBRARY})

endif()







