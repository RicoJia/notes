<?php
if( ! function_exists( 'burger_com_cozipress_dynamic_style' ) ):
    function burger_com_cozipress_dynamic_style() {

		$output_css = '';
		
		/**
		 * Logo Width 
		 */
		 $logo_width			= get_theme_mod('logo_width','140');		 
		if($logo_width !== '') { 
				$output_css .=".logo img, .mobile-logo img {
					max-width: " .esc_attr($logo_width). "px;
				}\n";
			}
			
		
		/**
		 *  Slider Style
		 */
		 $slider_opacity			= get_theme_mod('slider_opacity','0.75');	
		 $output_css .=".main-slider {
					    background: rgba(0, 0, 0, " .esc_attr($slider_opacity). ");
				}\n";
				
		 /**
		 *  Breadcrumb Style
		 */
		
		$breadcrumb_min_height			= get_theme_mod('breadcrumb_min_height','236');	
		
		if($breadcrumb_min_height !== '') { 
				$output_css .=".breadcrumb-area {
					min-height: " .esc_attr($breadcrumb_min_height). "px;
				}\n";
			}
		
		$breadcrumb_bg_img			= get_theme_mod('breadcrumb_bg_img',get_template_directory_uri() .'/assets/images/bg/breadcrumbg.jpg'); 
		$breadcrumb_bg_img_opacity	= get_theme_mod('breadcrumb_bg_img_opacity','0.75');
		
		if($breadcrumb_bg_img !== '') { 
			$output_css .=".breadcrumb-area:after {
					opacity: " .esc_attr($breadcrumb_bg_img_opacity). ";
				}\n";
		}		
		
		/**
		 *  Typography Body
		 */
		 $cozipress_body_text_transform	 	 = get_theme_mod('cozipress_body_text_transform','inherit');
		 $cozipress_body_font_style	 		 = get_theme_mod('cozipress_body_font_style','inherit');
		 $cozipress_body_font_size	 		 = get_theme_mod('cozipress_body_font_size','16');
		 $cozipress_body_line_height		 = get_theme_mod('cozipress_body_line_height','1.5');
		
		 $output_css .=" body{ 
			font-size: " .esc_attr($cozipress_body_font_size). "px;
			line-height: " .esc_attr($cozipress_body_line_height). ";
			text-transform: " .esc_attr($cozipress_body_text_transform). ";
			font-style: " .esc_attr($cozipress_body_font_style). ";
		}\n";		 
		
		/**
		 *  Typography Heading
		 */
		 for ( $i = 1; $i <= 6; $i++ ) {	
			 $cozipress_heading_text_transform 	= get_theme_mod('cozipress_h' . $i . '_text_transform','inherit');
			 $cozipress_heading_font_style	 	= get_theme_mod('cozipress_h' . $i . '_font_style','inherit');
			 $cozipress_heading_font_size	 		 = get_theme_mod('cozipress_h' . $i . '_font_size');
			 $cozipress_heading_line_height		 	 = get_theme_mod('cozipress_h' . $i . '_line_height');
			 
			 $output_css .=" h" . $i . "{ 
				font-size: " .esc_attr($cozipress_heading_font_size). "px;
				line-height: " .esc_attr($cozipress_heading_line_height). ";
				text-transform: " .esc_attr($cozipress_heading_text_transform). ";
				font-style: " .esc_attr($cozipress_heading_font_style). ";
			}\n";
		 }
		 
		 
		
        wp_add_inline_style( 'cozipress-style', $output_css );
    }
endif;
add_action( 'wp_enqueue_scripts', 'burger_com_cozipress_dynamic_style' );