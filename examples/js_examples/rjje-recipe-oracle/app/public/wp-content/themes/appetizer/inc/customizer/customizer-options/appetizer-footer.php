<?php
function appetizer_footer( $wp_customize ) {
$selective_refresh = isset( $wp_customize->selective_refresh ) ? 'postMessage' : 'refresh';
	// Footer Panel // 
	$wp_customize->add_panel( 
		'footer_section', 
		array(
			'priority'      => 34,
			'capability'    => 'edit_theme_options',
			'title'			=> __('Footer', 'appetizer'),
		) 
	);
	
	// Footer Background // 
	$wp_customize->add_section(
        'footer_background',
        array(
            'title' 		=> __('Footer Background','appetizer'),
			'panel'  		=> 'footer_section',
			'priority'      => 3,
		)
    );
	
	// Background // 
	$wp_customize->add_setting(
		'footer_bg_head'
			,array(
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_text',
			'priority' => 1,
		)
	);

	$wp_customize->add_control(
	'footer_bg_head',
		array(
			'type' => 'hidden',
			'label' => __('Background','appetizer'),
			'section' => 'footer_background',
		)
	);
	
	// Background Image // 
    $wp_customize->add_setting( 
    	'footer_bg_img' , 
    	array(
			'default' 			=> esc_url(get_template_directory_uri() .'/assets/images/footer/footer_bg.jpg'),
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_url',	
			'priority' => 10,
		) 
	);
	
	$wp_customize->add_control( new WP_Customize_Image_Control( $wp_customize , 'footer_bg_img' ,
		array(
			'label'          => esc_html__( 'Background Image', 'appetizer'),
			'section'        => 'footer_background',
		) 
	));
	
	// Background Attachment // 
	$wp_customize->add_setting( 
		'footer_back_attach' , 
			array(
			'default' => 'fixed',
			'capability'     => 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_select',
			'priority'  => 10,
		) 
	);
	
	$wp_customize->add_control(
	'footer_back_attach' , 
		array(
			'label'          => __( 'Background Attachment', 'appetizer' ),
			'section'        => 'footer_background',
			'type'           => 'select',
			'choices'        => 
			array(
				'inherit' => __( 'Inherit', 'appetizer' ),
				'scroll' => __( 'Scroll', 'appetizer' ),
				'fixed'   => __( 'Fixed', 'appetizer' )
			) 
		) 
	);
	
	// Footer Bottom // 
	$wp_customize->add_section(
        'footer_bottom',
        array(
            'title' 		=> __('Footer Bottom','appetizer'),
			'panel'  		=> 'footer_section',
			'priority'      => 3,
		)
    );

	// Footer Copyright Head
	$wp_customize->add_setting(
		'footer_btm_copy_head'
			,array(
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_text',
		)
	);

	$wp_customize->add_control(
	'footer_btm_copy_head',
		array(
			'type' => 'hidden',
			'label' => __('Copyright','appetizer'),
			'section' => 'footer_bottom',
			'priority'  => 1,
		)
	);
	
	// Footer Copyright 
	$appetizer_foo_copy = esc_html__('Copyright &copy; [current_year] [site_title] | Powered by [theme_author]', 'appetizer' );
	$wp_customize->add_setting(
    	'footer_copyright',
    	array(
			'default' => $appetizer_foo_copy,
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'wp_kses_post',
		)
	);	

	$wp_customize->add_control( 
		'footer_copyright',
		array(
		    'label'   		=> __('Copyright','appetizer'),
		    'section'		=> 'footer_bottom',
			'type' 			=> 'textarea',
			'transport'         => $selective_refresh,
			'priority'  => 1,
		)  
	);	
	
	
	// Footer Support 
	$wp_customize->add_setting(
		'footer_btm_support_head'
			,array(
			'capability'     	=> 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_text',
		)
	);

	$wp_customize->add_control(
	'footer_btm_support_head',
		array(
			'type' => 'hidden',
			'label' => __('Support','appetizer'),
			'section' => 'footer_bottom',
			'priority'  => 1,
		)
	);	
	
	$wp_customize->add_setting( 
		'hs_footer_btm_support' , 
			array(
			'default' => '1',
			'capability'     => 'edit_theme_options',
			'sanitize_callback' => 'appetizer_sanitize_checkbox',
		) 
	);
	
	$wp_customize->add_control(
	'hs_footer_btm_support', 
		array(
			'label'	      => esc_html__( 'Hide/Show', 'appetizer' ),
			'section'     => 'footer_bottom',
			'type'        => 'checkbox',
			'priority'  => 2,
		) 
	);	
	
	// icon // 
	$wp_customize->add_setting(
    	'footer_btm_support_icon',
    	array(
	        'default' => 'fa-phone',
			'sanitize_callback' => 'sanitize_text_field',
			'capability' => 'edit_theme_options',
		)
	);	

	$wp_customize->add_control(new Appetizer_Icon_Picker_Control($wp_customize, 
		'footer_btm_support_icon',
		array(
		    'label'   		=> __('Icon','appetizer'),
		    'section' 		=> 'footer_bottom',
			'iconset' => 'fa',
			'priority'  => 3,
			
		))  
	);	

	// Support Title // 
	$wp_customize->add_setting(
    	'footer_btm_support_ttl',
    	array(
			'sanitize_callback' => 'appetizer_sanitize_html',
			'transport'         => $selective_refresh,
			'capability' => 'edit_theme_options',
		)
	);	

	$wp_customize->add_control( 
		'footer_btm_support_ttl',
		array(
		    'label'   		=> __('Title','appetizer'),
		    'section' 		=> 'footer_bottom',
			'type'		 =>	'text',
			'priority' => 4,
		)  
	);	
	// Support Text // 
	$wp_customize->add_setting(
    	'footer_btm_support_text',
    	array(
			'sanitize_callback' => 'appetizer_sanitize_html',
			'transport'         => $selective_refresh,
			'capability' => 'edit_theme_options',
		)
	);	

	$wp_customize->add_control( 
		'footer_btm_support_text',
		array(
		    'label'   		=> __('Text','appetizer'),
		    'section' 		=> 'footer_bottom',
			'type'		 =>	'textarea',
			'priority' => 5,
		)  
	);
	
	
	
}
add_action( 'customize_register', 'appetizer_footer' );
// Footer selective refresh
function appetizer_footer_partials( $wp_customize ){
	
	// footer_btm_support_text
	$wp_customize->selective_refresh->add_partial( 'footer_btm_support_text', array(
		'selector'            => '.footer-copyright .widget-contact p',
		'settings'            => 'footer_btm_support_text',
		'render_callback'  => 'appetizer_footer_btm_support_text_render_callback',
	) );
	
	// footer_copyright
	$wp_customize->selective_refresh->add_partial( 'footer_copyright', array(
		'selector'            => '#footer-section .footer-copyright .copyright-text',
		'settings'            => 'footer_copyright',
		'render_callback'  => 'appetizer_footer_copyright_render_callback',
	) );
	
	}

add_action( 'customize_register', 'appetizer_footer_partials' );


// footer_btm_support_text
function appetizer_footer_btm_support_text_render_callback() {
	return get_theme_mod( 'footer_btm_support_text' );
}

// copyright_content
function appetizer_footer_copyright_render_callback() {
	return get_theme_mod( 'footer_copyright' );
}