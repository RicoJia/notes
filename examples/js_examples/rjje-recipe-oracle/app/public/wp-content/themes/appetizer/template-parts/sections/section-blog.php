<?php  
	$appetizer_hs_blog 			= get_theme_mod('hs_blog','1');
	$appetizer_blog_title 		= get_theme_mod('blog_title');
	$appetizer_blog_description	= get_theme_mod('blog_description');
	$appetizer_blog_display_num	= get_theme_mod('blog_display_num','3');
	if($appetizer_hs_blog=='1'):
?>		
<section id="post-section" class="post-section post-home st-py-default shapes-section">
	<div class="container">
		<?php if(!empty($appetizer_blog_title) || !empty($appetizer_blog_description)): ?>
			<div class="row">
				<div class="col-lg-6 col-12 mx-lg-auto mb-5 text-center">
					<div class="heading-default wow fadeInUp">
						<?php if(!empty($appetizer_blog_title)): ?>
							<h2><?php echo wp_kses_post($appetizer_blog_title); ?></h2>
						<?php endif; ?>
						<?php do_action('appetizer_section_seprator'); ?>
						<?php if(!empty($appetizer_blog_description)): ?>
							<p><?php echo wp_kses_post($appetizer_blog_description); ?></p>
						<?php endif; ?>
					</div>
				</div>
			</div>
		<?php endif; ?>
		<div class="row g-4">
			<?php 	
				$appetizer_blogs_args = array( 'post_type' => 'post', 'posts_per_page' => $appetizer_blog_display_num,'post__not_in'=>get_option("sticky_posts")) ; 	
				$appetizer_blog_wp_query = new WP_Query($appetizer_blogs_args);
				if($appetizer_blog_wp_query)
				{	
				while($appetizer_blog_wp_query->have_posts()):$appetizer_blog_wp_query->the_post();
			?>
			<div class="col-lg-4 col-md-6 col-12">
				<?php get_template_part('template-parts/content/content','page'); ?>
			</div>
			<?php 
				endwhile; 
				}
				wp_reset_postdata();
			?>
		</div>
	</div>
	<div class="lg-shape5 clipartss"><img src="<?php echo esc_url(get_template_directory_uri()); ?>/assets/images/clipArt/blog/shape5.png" alt="image"></div>
	<div class="lg-shape6 clipartss"><img src="<?php echo esc_url(get_template_directory_uri()); ?>/assets/images/clipArt/blog/shape6.png" alt="image"></div>
</section>
<?php endif; ?>