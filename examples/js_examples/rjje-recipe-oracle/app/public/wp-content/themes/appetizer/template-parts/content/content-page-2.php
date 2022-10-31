<?php
/**
 * Template part for displaying page content in page.php.
 *
 * @link https://codex.wordpress.org/Template_Hierarchy
 *
 * @package Appetizer
 */

?>
<article id="post-<?php the_ID(); ?>" <?php post_class('post-items post-items2'); ?>>
	<figure class="post-image">
		<div class="featured-image">
			<a href="<?php echo esc_url(the_permalink()); ?>" class="post-hover">
				<?php the_post_thumbnail(); ?>
			</a>
			<span class="post-date">
				<a href="<?php echo esc_url(get_month_link(get_post_time('Y'),get_post_time('m'))); ?>"><span><?php echo esc_html(get_the_date('j')); ?></span><br><?php echo esc_html(get_the_date('M'));  echo esc_html(get_the_date(' Y')); ?></a>
			</span>
		</div>
		<div class="post-content post-item-absolute">
			<div class="post-meta">
				<span class="post-date">
					<i class="fa fa-calendar"></i> <a href="<?php echo esc_url(get_month_link(get_post_time('Y'),get_post_time('m'))); ?>"><?php echo esc_html(get_the_date('j')); echo esc_html(get_the_date('M'));  echo esc_html(get_the_date(' Y')); ?></a>
				</span>
				<span class="post-categories">
					<i class="fa fa-tag"></i> <a href="<?php echo esc_url(get_permalink());?>"><?php the_category(', '); ?></a>
				</span>
			</div>
		</div>
	</figure>
	<div class="post-content">
		<?php 
			if ( is_single() ) :
				
			the_title('<h5 class="post-title">', '</h5>' );
			
			else:
			
			the_title( sprintf( '<h5 class="post-title"><a href="%s" rel="bookmark">', esc_url( get_permalink() ) ), '</a></h5>' );
			
			endif; 
			
			the_content( 
				sprintf( 
					__( 'Read More', 'appetizer' ), 
					'<span class="screen-reader-text">  '.esc_html(get_the_title()).'</span>' 
				) 
			);
			
		?>
	</div>
</article>