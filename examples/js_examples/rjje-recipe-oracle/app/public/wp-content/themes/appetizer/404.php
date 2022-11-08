<?php
/**
 * The template for displaying 404 pages (not found).
 *
 * @link https://codex.wordpress.org/Creating_an_Error_404_Page
 *
 * @package Appetizer
 */
 get_header();
?>
<section id="page404-section" class="page404-section st-py-default">
	<div class="container">
		<div class="row g-5 wow fadeIn">
			<div class="col-lg-12 col-md-12 col-12">
				<div class="page404">		
						<h2><?php echo esc_html_e('OOPs...','appetizer'); ?></h2> 
						<h4><?php echo esc_html_e('Page Not Found','appetizer'); ?></h4> 
					<h1><?php echo esc_html_e('4','appetizer'); ?><span class="page404icon"><img src="<?php echo esc_url(get_template_directory_uri()); ?>/assets/images/icon_gif/plate-fork-knife.gif"></span><?php echo esc_html_e('4','appetizer'); ?></h1>
					
					<div class="page404-btn">
						<a href="<?php echo esc_url( home_url( '/' ) ); ?>" class="btn btn-primary" data-text="<?php echo esc_attr_e('Go Home Page','appetizer'); ?>"><span><?php echo esc_html_e('Go Home Page','appetizer'); ?></span></a>
					</div>
				</div>
			</div>
		</div>
	</div>
</section>
<?php get_footer(); ?>
