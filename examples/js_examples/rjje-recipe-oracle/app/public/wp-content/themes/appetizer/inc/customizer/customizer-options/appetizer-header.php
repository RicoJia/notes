<?php
function appetizer_header_settings( $wp_customize ) {
$selective_refresh = isset( $wp_customize->selective_refresh ) ? 'postMessage' : 'refresh';
	/*=========================================
	Header Settings Panel
	=========================================*/
	$wp_customize->add_panel( 
		'header_section', 
		array(
			'priority'      => 2,
			'capability'    => 'edit_theme_options',
			'title'			=> __('Header', 'appetizer'),
		) 
	);
	
	/*=========================================
	Header Navigation
	=========================================*/	
	$wp_customize->add_section(
        'hdr_navigation',
        array(
        	'priority'      => 3,
            'title' 		=> __('Header Navigation','appetizer'),
			'panel'  		=> 'header_section',
		)
    );

    // Cart
	$wp_customize->add_setting(
		'hdr_nav_cart'
			,array(
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_text',
		)
	);

	$wp_customize->add_control(
	'hdr_nav_cart',
		array(
			'type' => 'hidden',
			'label' => __('Cart','appetizer'),
			'section' => 'hdr_navigation',
			'priority' => 2,
		)
	);

	// hide/show
	$wp_customize->add_setting( 
		'hide_show_cart' , 
			array(
			'default' => '1',
			'capability'     => 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_checkbox',
		) 
	);
	
	$wp_customize->add_control(
	'hide_show_cart', 
		array(
			'label'	      => esc_html__( 'Hide/Show', 'appetizer' ),
			'section'     => 'hdr_navigation',
			'type'        => 'checkbox',
			'priority' => 2,
		) 
	);
	
	// Header Hiring Section
	$wp_customize->add_setting(
		'hdr_nav_search_head'
			,array(
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_text',
		)
	);

	$wp_customize->add_control(
	'hdr_nav_search_head',
		array(
			'type' => 'hidden',
			'label' => __('Search','appetizer'),
			'section' => 'hdr_navigation',
			'priority'  => 3,
		)
	);	
	
	// hide/show
	$wp_customize->add_setting( 
		'hs_nav_search' , 
			array(
			'default' => '1',
			'capability'     => 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_checkbox',
		) 
	);
	
	$wp_customize->add_control(
	'hs_nav_search', 
		array(
			'label'	      => esc_html__( 'Hide/Show', 'appetizer' ),
			'section'     => 'hdr_navigation',
			'type'        => 'checkbox',
			'priority' => 3,
		) 
	);
	
	
	// Header Button 
	$wp_customize->add_setting(
		'abv_hdr_btn_head'
			,array(
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_text',
		)
	);

	$wp_customize->add_control(
	'abv_hdr_btn_head',
		array(
			'type' => 'hidden',
			'label' => __('Button','appetizer'),
			'section' => 'hdr_navigation',
			'priority'  => 15,
		)
	);	
	
	$wp_customize->add_setting( 
		'hide_show_hdr_btn' , 
			array(
			'capability'     => 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_checkbox',
		) 
	);
	
	$wp_customize->add_control(
	'hide_show_hdr_btn', 
		array(
			'label'	      => esc_html__( 'Hide/Show', 'appetizer' ),
			'section'     => 'hdr_navigation',
			'type'        => 'checkbox',
			'priority'  => 16,
		) 
	);	

	// Button Label // 
	$wp_customize->add_setting(
    	'hdr_btn_lbl',
    	array(
			'sanitize_callback' => 'appetizer_sanitize_text',
			'transport'         => $selective_refresh,
			'capability' => 'edit_theme_options',
		)
	);	

	$wp_customize->add_control( 
		'hdr_btn_lbl',
		array(
		    'label'   		=> __('Label','appetizer'),
		    'section' 		=> 'hdr_navigation',
			'type'		 =>	'text',
			'priority' => 18,
		)  
	);	
	
	// Button URL // 
	$wp_customize->add_setting(
    	'hdr_btn_url',
    	array(
			'sanitize_callback' => 'appetizer_sanitize_url',
			'capability' => 'edit_theme_options',
		)
	);	

	$wp_customize->add_control( 
		'hdr_btn_url',
		array(
		    'label'   		=> __('Link','appetizer'),
		    'section' 		=> 'hdr_navigation',
			'type'		 =>	'text',
			'priority' => 19,
		)  
	);	
	
	$wp_customize->add_setting( 
		'hdr_btn_open_new_tab' , 
			array(
			'capability'     => 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_checkbox',
		) 
	);
	
	$wp_customize->add_control(
	'hdr_btn_open_new_tab', 
		array(
			'label'	      => esc_html__( 'Open in New Tab ?', 'appetizer' ),
			'section'     => 'hdr_navigation',
			'type'        => 'checkbox',
			'priority'  => 19,
		) 
	);	
	/*=========================================
	Sticky Header
	=========================================*/	
	$wp_customize->add_section(
        'sticky_header_set',
        array(
        	'priority'      => 4,
            'title' 		=> __('Sticky Header','appetizer'),
			'panel'  		=> 'header_section',
		)
    );
	
	// Heading
	$wp_customize->add_setting(
		'sticky_head'
			,array(
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_text',
			'priority' => 1,
		)
	);

	$wp_customize->add_control(
	'sticky_head',
		array(
			'type' => 'hidden',
			'label' => __('Sticky Header','appetizer'),
			'section' => 'sticky_header_set',
		)
	);
	$wp_customize->add_setting( 
		'hide_show_sticky' , 
			array(
			'default' => '1',
			'capability'     => 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_checkbox',
			'priority' => 2,
		) 
	);
	
	$wp_customize->add_control(
	'hide_show_sticky', 
		array(
			'label'	      => esc_html__( 'Hide/Show', 'appetizer' ),
			'section'     => 'sticky_header_set',
			'type'        => 'checkbox'
		) 
	);	
}
add_action( 'customize_register', 'appetizer_header_settings' );


// Header selective refresh
function appetizer_header_partials( $wp_customize ){
	
	//hdr_btn_lbl
	$wp_customize->selective_refresh->add_partial( 'hdr_btn_lbl', array(
		'selector'            => '.navigation-wrapper .button-area a',
		'settings'            => 'hdr_btn_lbl',
		'render_callback'  => 'appetizer_hdr_btn_lbl_render_callback',
	) );
	}

add_action( 'customize_register', 'appetizer_header_partials' );


// hdr_btn_lbl
function appetizer_hdr_btn_lbl_render_callback() {
	return get_theme_mod( 'hdr_btn_lbl' );
}
