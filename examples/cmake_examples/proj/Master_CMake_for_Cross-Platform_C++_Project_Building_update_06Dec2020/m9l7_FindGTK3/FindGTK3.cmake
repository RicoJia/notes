#Define a function to wrap find_library() command
function(GTK3_FIND_LIBRARY library_variable library_name)
	
	find_library(${library_variable}
            NAMES ${library_name}
            HINTS
                /opt/gnome/lib
                /opt/gnome/lib64)

	set(${library_variable}  ${${library_variable}} PARENT_SCOPE)

endfunction()

#Define a function to wrap find_path() command
function(GTK3_FIND_INCLUDE_DIR path_variable header_file)

    set(DEPENDENCY
        gtk-3.0
	glib-2.0
        pango-1.0
	cairo
        gdk-pixbuf-2.0
        atk-1.0
    )

#   we want to look for headers inside the include directories of the above mentioned directories, as well
    set(SUFFIXES)
    foreach(suffix ${DEPENDENCY})
        list(APPEND SUFFIXES ${suffix})
        list(APPEND SUFFIXES ${suffix}/include)
    endforeach()


    find_path(${path_variable} ${header_file}
        HINTS
            /usr/local/lib64
            /usr/local/lib
            /usr/lib/i386-linux-gnu/
            /usr/lib/x86_64-linux-gnu/
            /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
            /usr/lib64
            /usr/lib
            /opt/gnome/include
            /opt/gnome/lib
            /opt/local/include
            /opt/local/lib
        PATH_SUFFIXES
            ${SUFFIXES})

    set(${path_variable}  ${${path_variable}} PARENT_SCOPE)

endfunction()


############### main() ###################

GTK3_FIND_LIBRARY    (GTK3_LIBRARY gtk-3)
GTK3_FIND_LIBRARY    (GIO_LIBRARY gio-2.0)
GTK3_FIND_LIBRARY    (GOBJECT_LIBRARY gobject-2.0)

GTK3_FIND_INCLUDE_DIR(GTK3_INCLUDE_DIR gtk/gtk.h)
GTK3_FIND_INCLUDE_DIR(GLIB_INCLUDE_DIR glib.h)
GTK3_FIND_INCLUDE_DIR(GLIBCONFIG_INCLUDE_DIR glibconfig.h)
GTK3_FIND_INCLUDE_DIR(PANGO_INCLUDE_DIR pango/pango.h)
GTK3_FIND_INCLUDE_DIR(CAIRO_INCLUDE_DIR cairo.h)
GTK3_FIND_INCLUDE_DIR(GDK_PIXBUF_INCLUDE_DIR gdk-pixbuf/gdk-pixbuf.h)
GTK3_FIND_INCLUDE_DIR(ATK_INCLUDE_DIR atk/atk.h)

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

if(GTK3_INCLUDE_DIRS)
   list(REMOVE_DUPLICATES GTK3_INCLUDE_DIRS)
endif()