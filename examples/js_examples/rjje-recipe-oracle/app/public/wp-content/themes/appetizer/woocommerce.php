<?php
/**
 * The template for displaying all single posts.
 *
 * @link https://developer.wordpress.org/themes/basics/template-hierarchy/#single-post
 *
 * @package Appetizer
 */

get_header();
?>
<!-- Product Sidebar Section -->
<section id="product" class="product-section woo-shop st-py-default">
        <div class="container">
            <div class="row gy-lg-0 gy-5 wow fadeInUp">
			<!--Product Detail-->
			<?php if ( ! is_active_sidebar( 'appetizer-woocommerce-sidebar' ) ) {	?>
				<div id="product-content" class="col-lg-12">
			<?php }else{ ?>
				<div id="product-content" class="col-lg-8">
			<?php } ?>	
				<?php woocommerce_content(); ?>
			</div>
			<!--/End of Blog Detail-->
			<?php get_sidebar('woocommerce'); ?>
		</div>	
	</div>
</section>
<!-- End of Blog & Sidebar Section -->
<?php get_footer(); ?>

