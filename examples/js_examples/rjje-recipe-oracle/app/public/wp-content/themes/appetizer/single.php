<?php
/**
 * The template for displaying archive pages.
 *
 * @link https://codex.wordpress.org/Template_Hierarchy
 *
 * @package Appetizer
 */

get_header();
?>
<section id="post-section" class="post-section st-py-default">
	<div class="container">
		<div class="row g-5 wow fadeIn">
			<div class="col-lg-8 col-12">
				<div class="row g-5">
					<div class="col-lg-12 col-md-12 col-12">
						<?php if( have_posts() ): ?>
							<?php while( have_posts() ): the_post(); ?>
								<article class="post-items blog-single">
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
													<i class="fa fa-calendar-alt"></i> <a href="<?php echo esc_url(get_month_link(get_post_time('Y'),get_post_time('m'))); ?>"><?php echo esc_html(get_the_date('j')); echo esc_html(get_the_date('M'));  echo esc_html(get_the_date(' Y')); ?></a>
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
                                        <p>Steps: <?php echo get_post_meta($post->ID, 'steps', true); ?></p>
                                        <p>Ingredients: 
                                            <?php 
                                                $table = get_field( 'ingredients' );
                                                if ( ! empty ( $table ) ) {
                                                    echo '<table border="0">';
                                                        if ( ! empty( $table['caption'] ) ) {
                                                            echo '<caption>' . $table['caption'] . '</caption>';
                                                        }
                                                        if ( ! empty( $table['header'] ) ) {
                                                            echo '<thead>';
                                                                echo '<tr>';
                                                                    foreach ( $table['header'] as $th ) {
                                                                        echo '<th>';
                                                                            echo $th['c'];
                                                                        echo '</th>';
                                                                    }
                                                                echo '</tr>';
                                                            echo '</thead>';
                                                        }
                                                        echo '<tbody>';
                                                            foreach ( $table['body'] as $tr ) {
                                                                echo '<tr>';
                                                                    foreach ( $tr as $td ) {
                                                                        echo '<td> - ';
                                                                            echo $td['c'];
                                                                        echo '</td>';
                                                                    }
                                                                echo '</tr>';
                                                            }
                                                        echo '</tbody>';
                                                    echo '</table>';
                                                }
                                            ?> 
                                        </p>
                                        <p>Rating: <?php the_field('rating');?> </p>
									</div>
								</article>
							<?php endwhile; ?>
						<?php endif; ?>
						<?php comments_template( '', true ); // show comments  ?>
					</div>
				</div>
			</div>
			<?php get_sidebar(); ?>
		</div>
	</div>
</section>
<?php get_footer(); ?>
